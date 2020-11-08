
/**
 * Mostly copied from https://github.com/br3ttb/Arduino-PID-AutoTune-Library
 */
#ifndef LIBPIDAUTOTUNE_PID_AUTOTUNE_H
#define LIBPIDAUTOTUNE_PID_AUTOTUNE_H

#include <stdint.h>

namespace PIDAutotune {
    class Autotune {
    public:

        enum ControlType {
            NoneD,
            IncludeD
        };

        enum ParameterSearchErrors {
            Done,
            Busy,
            Timeout,
            NotReachable
        };

        struct TuningParameters {
            TuningParameters();
            double setpoint, step, start, noise, maxOutput;
            int loopbackSec;
            ControlType ctrlType;
        };

        struct TuningParameterSearchConfig {
            TuningParameterSearchConfig();

            double increaseStep; // Value which will be added to previous output when target and slope not reached
            double minSlopeOvershoot; // the slope which must be exceed when target reached and beeing in overshoot
            double minSlopeUntilIncreaseStep; // minimum slope until increasing output by <step>

            double decreaseStep; // Value which will be subtracted to previous output when below target and slope not reached
            double minSlopeBelowTarget; // The slope which must be exceeded when below target
            double minSlopeUntilDecreaseStep; // minimum slope until decreasing output by <step>

            double startOutput; // Sets the initial output where the process will start from, should be rather low !
            double slopeLoopbackMilli; // Time in milliseconds looking back for calculating slope
            double settleTimeoutSec; // Time in seconds to be waiting to let the input settle

            uint8_t timeoutMin; // Timeout in minutes, when exceeded finding parameters returns error
        };

        Autotune(double* input, double* output, double setpoint);
        Autotune(TuningParameters &tuningParameters);

        int Runtime();// * Similar to the PID Compue function, returns non 0 when done
        void Cancel();// * Stops the AutoTune

        void SetOutputStep(double);// * how far above and below the starting value will the output step?
        double GetOutputStep();

        void SetControlType(int);// * Determies if the tuning parameters returned will be PI (D=0)
        int GetControlType();//   or PID.  (0=PI, 1=PID)

        void SetLookbackSec(int);// * how far back are we looking to identify peaks
        int GetLookbackSec();

        void SetNoiseBand(double);// * the autotune will ignore signal chatter smaller than this value
        double GetNoiseBand();//   this should be acurately set

        double GetKp();// * once autotune is complete, these functions contain the
        double GetKi();//   computed tuning parameters.
        double GetKd();

        /**
         * Tries to find the best start parameters for tuning process
         *
         * The idea is to slowly raise the output so the target is reached and has a certain slope
         * If an appropriate value is found, the output is slowly decreased until the input is below target with
         * a certain slope. When both values are found the start and step values are calculated
         * and set to tuningParameters.
         *
         * with this configuration, the actual tuning can kick in.
         *
         * @return  Busy == Still in Progress, Timeout == timeout has been reached without success
         *          NotReachable == Cant reach target in respect to maxOutput
         */
        ParameterSearchErrors findTuningParameters(uint8_t timeoutInMinutes);
        ParameterSearchErrors findTuningParameters(TuningParameterSearchConfig &config);

    private:

        enum ParameterSearchMode {
            High, // finding the overshoot point
            Low, // Finding the point below target
            Settle // special timeout which gives a little bit time to settle the input
        };

        struct ParameterSearchData {
            ParameterSearchData();
            void clean();
            ParameterSearchMode mode;
            ParameterSearchMode lastMode; // for restoring last mode

            bool initialized; // Indicates if its the first run, if not the actual process begins
            double lastPoint;
            unsigned long start; // the time in minutes from epoch when process has been started, needed for timeout
            unsigned long lastCapture;
            unsigned long startSettle; // Time in milliseconds when settle started

            double highOutput; // Value which reaches target and high slope
            double lowOutput; // Value which results in <input> lower than target with respect to lowSlope
        };

        void FinishUp();

        TuningParameters *parameters;
        TuningParameterSearchConfig *searchConfig;
        ParameterSearchData searchData;


        bool isMax, isMin;
        double *input, *output;
        double setpoint;
        double noiseBand;
        int controlType;
        bool running;
        unsigned long peak1, peak2, lastTime;
        int sampleTime;
        int nLookBack;
        int peakType;
        double lastInputs[101];
        double peaks[10];
        int peakCount;
        bool justchanged;
        bool justevaled;
        double absMax, absMin;
        double oStep;
        double outputStart;
        double Ku, Pu;
        static unsigned long millis();
        static unsigned long minutes();

    };
}

#endif //LIBPIDAUTOTUNE_PID_AUTOTUNE_H
