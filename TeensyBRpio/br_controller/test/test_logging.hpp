#ifndef _TEST_LOGGING_HPP_
#define _TEST_LOGGING_HPP_

#include "logging.hpp"

extern unsigned int DISABLE_LOGGING;
extern LogSeverity MIN_SEVERITY;

/// RAII guard that disables logging until it is dropped.
/// Can be nested.
struct DisableLoggingGuard {
  public:
    DisableLoggingGuard();
    ~DisableLoggingGuard();
};

#endif