#ifndef _DEFINE_LOGGING_HPP_
#define _DEFINE_LOGGING_HPP_

#include "defines/string.hpp"

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

void log(LogSeverity severity, const char *message);

[[noreturn]] inline void abort(const char *message) {
    log(FATAL, message);
#ifdef __EXCEPTIONS
    throw std::runtime_error(message);
#else
    std::abort();
#endif
}

#ifdef _STRING_EXT_
inline void log(LogSeverity severity, const std::string &message) {
    log(severity, message.c_str());
}

[[noreturn]] inline void abort(const std::string &message) {
    abort(message.c_str());
}
#endif

#endif