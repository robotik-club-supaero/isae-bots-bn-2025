#ifndef _ROS_MSG_EMPTY_HPP_
#define _ROS_MSG_EMPTY_HPP_

#include <std_msgs/msg/empty.h>

#include "ros2/std_msgs/__defines.hpp"

namespace std_msgs {

struct Empty : public std_msgs__msg__Empty {
   public:
    Empty() = default;

    static support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty); }
};

}  // namespace std_msgs

#endif