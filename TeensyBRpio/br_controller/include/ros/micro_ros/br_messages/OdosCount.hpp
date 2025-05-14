#ifndef _BR_MESSAGES_MSG_ODOS_COUNT_HPP_
#define _BR_MESSAGES_MSG_ODOS_COUNT_HPP_

#include "br_messages/msg/odos_count.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class OdosCount : public br_messages__msg__OdosCount {
  public:
    OdosCount() = default;
    OdosCount(int32_t left, int32_t right) {
        this->left = left;
        this->right = right;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, OdosCount); }
};
} // namespace msg

} // namespace br_messages

#endif