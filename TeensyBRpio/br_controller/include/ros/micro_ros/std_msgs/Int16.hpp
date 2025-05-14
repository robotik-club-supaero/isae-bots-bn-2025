#ifndef _STD_MSGS_MSG_INT16_HPP_
#define _STD_MSGS_MSG_INT16_HPP_

#include "ros/micro_ros/type_support.hpp"

#include <std_msgs/msg/int16.h>

namespace std_msgs {
namespace msg {

class Int16 : public std_msgs__msg__Int16 {
  public:
    Int16() = default;
    Int16(int16_t value) { data = value; }

    operator int16_t() const { return data; }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16); }
};

} // namespace msg
} // namespace std_msgs

#endif