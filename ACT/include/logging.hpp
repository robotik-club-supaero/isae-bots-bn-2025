#ifndef _DEFINE_LOGGING_HPP_
#define _DEFINE_LOGGING_HPP_

#include <Arduino.h>

enum LogSeverity {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

// NB: Defined in `main.cpp`.
/// Sends a log message to ROS. This currently publishes to /act/logging instead of /rosout, but this could be changed in the future.
void log(LogSeverity severity, String message);

#endif