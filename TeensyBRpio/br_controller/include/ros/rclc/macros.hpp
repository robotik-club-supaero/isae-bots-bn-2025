#ifndef _ROS_IMPL_MACROS_HPP_
#define _ROS_IMPL_MACROS_HPP_

#ifdef ARDUINO
#include <Arduino.h>
#define ROS_REPORT /* Nothing to do */
#define TEENSY_RESTART SCB_AIRCR = 0x05FA0004
#define ROS_ABORT                                                                                                                                    \
    delay(2000);                                                                                                                                     \
    TEENSY_RESTART;
#else
#include <iostream>
#define ROS_REPORT std::cout << "ROS error at " << __FILE__ << ":" << __LINE__ << ": " << rcutils_get_error_string().str << std::endl;
#define ROS_ABORT std::abort()
#endif

#define RCCHECK_HARD(fn)                                                                                                                             \
    {                                                                                                                                                \
        if ((fn) != RCL_RET_OK) {                                                                                                                    \
            ROS_REPORT;                                                                                                                              \
            ROS_ABORT;                                                                                                                               \
        }                                                                                                                                            \
    }

#define RCCHECK_SOFT(fn)                                                                                                                             \
    {                                                                                                                                                \
        if ((fn) != RCL_RET_OK) {                                                                                                                    \
            ROS_REPORT;                                                                                                                              \
        }                                                                                                                                            \
    }

#endif