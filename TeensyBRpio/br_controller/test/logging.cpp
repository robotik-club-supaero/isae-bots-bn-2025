#include "logging.hpp"
#include <iostream>

void log(LogSeverity severity, const string_t &message) {
    std::cout << severityToString(severity) << " " << message << std::endl;
}