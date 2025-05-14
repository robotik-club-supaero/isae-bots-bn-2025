#ifndef _BR_MESSAGES_MSG_LOG_ENTRY_HPP_
#define _BR_MESSAGES_MSG_LOG_ENTRY_HPP_

#include "br_messages/msg/log_entry.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class LogEntry : public br_messages__msg__LogEntry {
  public:
    LogEntry() = default;

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, LogEntry); }
};
} // namespace msg

} // namespace br_messages

#endif