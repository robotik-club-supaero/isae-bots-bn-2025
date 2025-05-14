#ifndef _STD_MSGS_MSG_EMPTY_HPP_
#define _STD_MSGS_MSG_EMPTY_HPP_

#include "ros/micro_ros/type_support.hpp"

#include <std_msgs/msg/empty.h>

namespace std_msgs {
namespace msg {

class Empty : public std_msgs__msg__Empty {
  public:
    Empty() = default;

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty); }
};

} // namespace msg
} // namespace std_msgs

#endif