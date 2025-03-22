#ifndef _ROS_IMPL_MACROS_HPP_
#define _ROS_IMPL_MACROS_HPP_

#include <Arduino.h>

#define TEENSY_RESTART SCB_AIRCR = 0x05FA0004

#define RCCHECK_HARD(fn)                                                                                                                             \
    {                                                                                                                                                \
        rcl_ret_t temp_rc = fn;                                                                                                                      \
        if ((temp_rc != RCL_RET_OK)) {                                                                                                               \
            delay(2000);                                                                                                                             \
            TEENSY_RESTART;                                                                                                                          \
        }                                                                                                                                            \
    }

#define RCCHECK_SOFT(fn)                                                                                                                             \
    {                                                                                                                                                \
        std::ignore = fn;                                                                                                                            \
    }

#endif