#ifndef _DEFINE_LOGGING_HPP_
#define _DEFINE_LOGGING_HPP_

#include "defines/string.h"

#ifdef __EXCEPTIONS
#include <stdexcept>
#else
#include <cstdlib>
#endif

enum LogSeverity {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

inline string_t severityToString(LogSeverity severity) {
    switch (severity) {
        case DEBUG:
            return "DEBUG";
        case INFO:
            return "INFO";
        case WARN:
            return "WARN";
        case ERROR:
            return "ERROR";
        case FATAL:
            return "FATAL";
        default:
            return "INVALID";
    }
}

void log(LogSeverity severity, const string_t &message);

[[noreturn]] inline void abort(const string_t &message) {
    log(FATAL, message);
#ifdef __EXCEPTIONS
    throw std::runtime_error(message);
#else
    std::abort();
#endif
}

#endif