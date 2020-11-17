#include "pid_autotune.h"
#include <chrono>
#include <cstdlib>
#include <stdio.h>

using namespace PIDAutotune;


Autotune::Autotune(Autotune::TuningParameters &tuningParameters) {
    this->parameters = &tuningParameters;
    running = false;
    SetLookbackSec(parameters->loopbackSec);
    lastTime = Autotune::millis();
    searchConfig = nullptr;
}

void Autotune::Cancel() {
    running = false;
}

int Autotune::Runtime() {
    justevaled = false;
    if (peakCount > 9 && running) {
        running = false;
        FinishUp();
        return 1;
    }
    unsigned long now = Autotune::millis();

    if ((now - lastTime) < sampleTime) return false;
    lastTime = now;
    double refVal = *parameters->input;
    justevaled = true;
    if (!running) { //initialize working variables the first time around
        peakType = 0;
        peakCount = 0;
        justchanged = false;
        absMax = refVal;
        absMin = refVal;
        running = true;
        outputStart = parameters->start;
        *parameters->output = outputStart + parameters->step;
    } else {
        if (refVal > absMax)absMax = refVal;
        if (refVal < absMin)absMin = refVal;
    }

    //oscillate the output base on the input's relation to the setpoint

    if (refVal > parameters->setpoint + parameters->noise) *parameters->output = outputStart - parameters->step;
    else if (refVal < parameters->setpoint - parameters->noise) *parameters->output = outputStart + parameters->step;


    //bool isMax=true, isMin=true;
    isMax = true;
    isMin = true;
    //id peaks
    for (int i = nLookBack - 1; i >= 0; i--) {
        double val = lastInputs[i];
        if (isMax) isMax = refVal > val;
        if (isMin) isMin = refVal < val;
        lastInputs[i + 1] = lastInputs[i];
    }
    lastInputs[0] = refVal;
    if (nLookBack < 9) {  //we don't want to trust the maxes or mins until the inputs array has been filled
        return 0;
    }

    if (isMax) {
        if (peakType == 0)peakType = 1;
        if (peakType == -1) {
            peakType = 1;
            justchanged = true;
            peak2 = peak1;
        }
        peak1 = now;
        peaks[peakCount] = refVal;

    } else if (isMin) {
        if (peakType == 0)peakType = -1;
        if (peakType == 1) {
            peakType = -1;
            peakCount++;
            justchanged = true;
        }

        if (peakCount < 10)peaks[peakCount] = refVal;
    }

    if (justchanged && peakCount > 2) { //we've transitioned.  check if we can autotune based on the last peaks
        double avgSeparation =
                (abs(peaks[peakCount - 1] - peaks[peakCount - 2]) + abs(peaks[peakCount - 2] - peaks[peakCount - 3])) /
                2;
        if (avgSeparation < 0.05 * (absMax - absMin)) {
            FinishUp();
            running = false;
            return 1;

        }
    }
    justchanged = false;
    return 0;
}

void Autotune::FinishUp() {
    *parameters->output = outputStart;
    //we can generate tuning parameters!
    Ku = 4 * (2 * parameters->step) / ((absMax - absMin) * 3.14159);
    Pu = (double) (peak1 - peak2) / 1000;
}

double Autotune::GetKp() {
    return parameters->ctrlType == IncludeD ? 0.6 * Ku : 0.4 * Ku;
}

double Autotune::GetKi() {
    return parameters->ctrlType == IncludeD ? 1.2 * Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

double Autotune::GetKd() {
    return parameters->ctrlType == IncludeD ? 0.075 * Ku * Pu : 0;  //Kd = Kc * Td
}


void Autotune::SetLookbackSec(int value) {
    if (value < 1) value = 1;

    if (value < 25) {
        nLookBack = value * 4;
        sampleTime = 250;
    } else {
        nLookBack = 100;
        sampleTime = value * 10;
    }
}

int Autotune::GetLookbackSec() {
    return nLookBack * sampleTime / 1000;
}

unsigned long Autotune::millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

unsigned long Autotune::minutes() {
    return std::chrono::duration_cast<std::chrono::minutes>(
            std::chrono::system_clock::now().time_since_epoch()
    ).count();
}


Autotune::ParameterSearchErrors Autotune::findTuningParameters(uint8_t timeoutInMinutes) {
    if (this->searchConfig == nullptr) {
        this->searchConfig = new TuningParameterSearchConfig();
        this->searchConfig->timeoutMin = timeoutInMinutes;
    }
    return findTuningParameters(*searchConfig);
}

Autotune::ParameterSearchErrors Autotune::findTuningParameters(Autotune::TuningParameterSearchConfig &config) {

    if (!searchData.initialized) {
        // First time called, initialize data, thus a valid datapoint is present on next call
        searchData.start = minutes();
        searchData.lastCapture = millis();
        searchData.lastPoint = *parameters->input;
        searchData.initialized = true;
        // Set the output to start value
        *parameters->output = config.startOutput;
        return Busy;
    }

    // Check if timeout exceeded
    auto isTimeout = (minutes() - searchData.start) >= config.timeoutMin;
    if (isTimeout) {
        // Timeout reached cleanup data and return the error
        searchData.clean();
        return Autotune::ParameterSearchErrors::Timeout;
    }

    // if the target is way too far away we do baning !
    if (!searchData.bangBangDone) {
        bool bangRes = bangBangToTarget(config);
        if (!bangRes) {
            return Busy;
        }
        searchData.bangBangDone = true;
    }



    // Copy the current input, so it does not change within processing
    double cInput = *parameters->input;
    auto slope = cInput - searchData.lastPoint;
    bool isCooling = slope < 0;
    if (slope < 0) {
        // always work with positive slope, cause negative slope is indicated by <isCooling>
        slope *= -1;
    }
    bool slopeOk, doCapture;
    // Processing
    switch (searchData.mode) {
        case High:
            if (isCooling) {
                // Check if just noise
                if (slope > searchConfig->coolingNoise) {
                    // Cooling is exactly the what we dont want, thus increase and set mode to settle
                    auto newOutput = *parameters->output + (searchConfig->increaseStep * 4);
                    if (newOutput >= parameters->maxOutput) {
                        // Still cooling and already exceeding maximum == Error
                        // reset output and return error
                        *parameters->output = 0;
                        searchData.clean();
                        return NotReachable;
                    }
                    *parameters->output = newOutput;
                    searchData.mode = Settle;
                    searchData.startSettle = millis();
                    searchData.lastMode = High;
                    break;
                }
            }
            // Capture data
            doCapture = (millis() - searchData.lastCapture) >= config.slopeLoopbackMilli;
            if (!doCapture) return Busy; // wait
            // Check if slope is within the limits, don't increase when target is not reached but slope is ok
            // Might be a slow process but rather save
            slopeOk = slope >= searchConfig->minSlopeUntilIncreaseStep;

            if (slopeOk && cInput < parameters->setpoint) break;
            // Check if target already reached, if so, than check slope overshoot
            if (cInput >= parameters->setpoint) {
                if (slope < searchConfig->minSlopeOvershoot) {
                    auto newOutput = *parameters->output + searchConfig->increaseStep;
                    if (newOutput >= parameters->maxOutput) {
                        // Still cooling and already exceeding maximum == Error
                        // reset output and return error
                        *parameters->output = 0;
                        searchData.clean();
                        return NotReachable;
                    }
                    *parameters->output = newOutput;
                } else {
                    // switch mode, High mode done
                    searchData.mode = Low;
                    searchData.highOutput = *parameters->output;
                }
            } else {
                // Slope is below cfg, thus increase output and continue
                auto newOutput = *parameters->output + searchConfig->increaseStep;
                if (newOutput >= parameters->maxOutput) {
                    // Still cooling and already exceeding maximum == Error
                    // reset output and return error
                    *parameters->output = 0;
                    searchData.clean();
                    return NotReachable;
                }

                *parameters->output = newOutput;
            }
            break;
        case Low:
            if (!isCooling) {
                // Wrong direction
                if (slope > searchConfig->coolingNoise) {
                    auto newOutput = *parameters->output - (searchConfig->increaseStep * 4);
                    if (newOutput < 0) {
                        // We cant do any better than 0 thus error!
                        *parameters->output = 0;
                        searchData.clean();
                        return NotReachable;
                    }
                    *parameters->output = newOutput;
                    searchData.mode = Settle;
                    searchData.startSettle = millis();
                    searchData.lastMode = Low;
                    break;
                }
            }

            // Capture data
            doCapture = (millis() - searchData.lastCapture) >= config.slopeLoopbackMilli;
            if (!doCapture) return Busy; // wait
            // Check if slope is within the limits, don't decrease when not below target but slope is ok
            // Might be a slow process but rather save
            slopeOk = slope >= searchConfig->minSlopeUntilDecreaseStep;
            if (slopeOk && cInput > parameters->setpoint) break;

            // Check if already below target, if so, than check slope
            if (cInput < parameters->setpoint) {
                if (slope < searchConfig->minSlopeBelowTarget) {
                    auto newOutput = *parameters->output - searchConfig->decreaseStep;
                    if (newOutput < 0) {
                        // We cant do any better than 0 thus error!
                        *parameters->output = 0;
                        searchData.clean();
                        return NotReachable;
                    }
                    *parameters->output = newOutput;
                } else {
                    // Low found reset output, set result and signal we are done
                    searchData.lowOutput = *parameters->output;
                    // Calculate step and start
                    parameters->start = (searchData.lowOutput + searchData.highOutput) / 2;
                    parameters->step = (double) searchData.highOutput - parameters->start;
                    *parameters->output = 0;
                    printf("Start %.2f step: %.2f\n", parameters->start, parameters->step);

                    return Done;
                }
            } else {
                // Slope is below cfg, thus increase output and continue
                auto newOutput = *parameters->output - searchConfig->decreaseStep;
                if (newOutput < 0) {
                    // We cant do any better than 0 thus error!
                    *parameters->output = 0;
                    searchData.clean();
                    return NotReachable;
                }
                *parameters->output = newOutput;
            }

            break;
        case Settle:
            bool settleTimeout =
                    ((double) (millis() - searchData.startSettle) / 1000) >= searchConfig->settleTimeoutSec;
            if (settleTimeout) searchData.mode = searchData.lastMode; // Reset mode
            break;
    }
    searchData.lastPoint = cInput;
    searchData.lastCapture = millis();

    return Busy;
}

bool Autotune::bangBangToTarget(Autotune::TuningParameterSearchConfig &config) {

    if (*parameters->input >= parameters->setpoint)
        return true;

    auto diff = parameters->setpoint - *parameters->input;

    if (*parameters->input >= (parameters->setpoint - config.maxBangBangDeviation)) {
        *parameters->output = config.startOutput;
        return true;
    } else {
        *parameters->output = parameters->maxOutput;
    }
    return false;
}


Autotune::TuningParameters::TuningParameters() : start(1),
                                                 step(0),
                                                 noise(1),
                                                 setpoint(0),
                                                 ctrlType(NoneD),
                                                 loopbackSec(20),
                                                 maxOutput(100),
                                                 input(nullptr),
                                                 output(nullptr) {}

Autotune::TuningParameterSearchConfig::TuningParameterSearchConfig() : increaseStep(0.8),
                                                                       minSlopeOvershoot(0.2),
                                                                       minSlopeUntilIncreaseStep(0.2),
                                                                       decreaseStep(0.8),
                                                                       minSlopeBelowTarget(0.05),
                                                                       minSlopeUntilDecreaseStep(0.05),
                                                                       slopeLoopbackMilli(4000),
                                                                       timeoutMin(60),
                                                                       settleTimeoutSec(2),
                                                                       startOutput(5),
                                                                       maxBangBangDeviation(3),
                                                                       coolingNoise(0.05) {
}

Autotune::ParameterSearchData::ParameterSearchData() : initialized(false),
                                                       lastPoint(0),
                                                       start(0),
                                                       mode(High),
                                                       lastCapture(0),
                                                       lowOutput(0),
                                                       highOutput(0),
                                                       lastMode(High),
                                                       startSettle(0),
                                                       bangBangDone(false) {
}

void Autotune::ParameterSearchData::clean() {
    initialized = false;
    lastPoint = 0;
    start = 0;
    lastCapture = 0;
    bangBangDone = false;
    mode = High;
}
