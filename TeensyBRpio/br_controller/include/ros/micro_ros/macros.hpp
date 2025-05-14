#ifndef _ROS_IMPL_MACROS_HPP_
#define _ROS_IMPL_MACROS_HPP_

#include <Arduino.h>

#define ROS_REPORT /* Nothing to do */
#define TEENSY_RESTART SCB_AIRCR = 0x05FA0004
#define ROS_ABORT                                                                                                                                    \
    delay(2000);                                                                                                                                     \
    TEENSY_RESTART;

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