#ifndef _STD_MSGS_MSG_BOOL_HPP_
#define _STD_MSGS_MSG_BOOL_HPP_

#include "ros/micro_ros/type_support.hpp"

#include <std_msgs/msg/bool.h>

namespace std_msgs {
namespace msg {

class Bool : public std_msgs__msg__Bool {
  public:
    Bool() = default;
    Bool(bool value) { data = value; }

    operator bool() const { return data; }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool); }
};

} // namespace msg
} // namespace std_msgs

#endif