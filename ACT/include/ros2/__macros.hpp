#ifndef _ROS_MACROS_HPP_
#define _ROS_MACROS_HPP_

#define RCCHECK_HARD(fn)          \
    {                             \
        if ((fn) != RCL_RET_OK) { \
            std::abort();            \
        }                         \
    }

#define RCCHECK_SOFT(fn)  \
    {                     \
        std::ignore = fn; \
    }

#endif