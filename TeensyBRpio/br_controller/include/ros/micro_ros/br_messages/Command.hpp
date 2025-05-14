#ifndef _BR_MESSAGES_MSG_COMMAND_HPP_
#define _BR_MESSAGES_MSG_COMMAND_HPP_

#include "br_messages/msg/command.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class Command : public br_messages__msg__Command {
  public:
    Command() = default;
    Command(double_t linear, double_t angular) {
        this->linear = linear;
        this->angular = angular;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, Command); }
};
} // namespace msg

} // namespace br_messages

#endif