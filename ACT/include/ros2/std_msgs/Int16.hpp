#ifndef _ROS_MSG_INT16_HPP_
#define _ROS_MSG_INT16_HPP_

#include <std_msgs/msg/int16.h>

#include "ros2/std_msgs/__defines.hpp"

namespace std_msgs {

struct Int16 : public std_msgs__msg__Int16 {
   public:
    Int16(uint16_t data) : std_msgs__msg__Int16() { this->data = data; }
    Int16() = default;

    static support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16); }
};

}  // namespace std_msgs

#endif