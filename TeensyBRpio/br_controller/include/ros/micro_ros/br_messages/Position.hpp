#ifndef _BR_MESSAGES_MSG_POSITION_HPP_
#define _BR_MESSAGES_MSG_POSITION_HPP_

#include "br_messages/msg/position.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class Position : public br_messages__msg__Position {
  public:
    Position() = default;
    Position(double_t x, double_t y, double_t theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, Position); }
};
} // namespace msg

} // namespace br_messages

#endif