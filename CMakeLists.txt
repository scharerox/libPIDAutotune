cmake_minimum_required(VERSION 3.17)
project(libPIDAutotune)

set(CMAKE_CXX_STANDARD 14)

add_library(libPIDAutotune SHARED pid_autotune.cpp pid_autotune.h)