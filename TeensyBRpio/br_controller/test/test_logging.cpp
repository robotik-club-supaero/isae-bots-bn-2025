#include "test_logging.hpp"
#include <iostream>

unsigned int DISABLE_LOGGING = 0;
LogSeverity MIN_SEVERITY = DEBUG;

void log(LogSeverity severity, const char *message) {
    if (!DISABLE_LOGGING && severity >= MIN_SEVERITY) {
        std::cout << severityToString(severity) << " " << message << std::endl;
    }
}

DisableLoggingGuard::DisableLoggingGuard() {
    ++DISABLE_LOGGING;
}
DisableLoggingGuard::~DisableLoggingGuard() {
    --DISABLE_LOGGING;
}
