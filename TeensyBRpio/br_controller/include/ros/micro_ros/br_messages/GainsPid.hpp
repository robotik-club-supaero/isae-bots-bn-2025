#ifndef _BR_MESSAGES_MSG_GAINS_PID_HPP_
#define _BR_MESSAGES_MSG_GAINS_PID_HPP_

#include "br_messages/msg/gains_pid.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class GainsPid : public br_messages__msg__GainsPid {
  public:
    GainsPid() = default;
    GainsPid(double_t kp, double_t ti, double_t td) {
        this->kp = kp;
        this->ti = ti;
        this->td = td;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, GainsPid); }
};
} // namespace msg

} // namespace br_messages

#endif