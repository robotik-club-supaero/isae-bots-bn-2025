#ifndef _BR_MESSAGES_MSG_DISP_ORDER_HPP_
#define _BR_MESSAGES_MSG_DISP_ORDER_HPP_

#include "br_messages/msg/displacement_order.h"
#include "ros/micro_ros/br_messages/Point.hpp"
#include "ros/micro_ros/macros.hpp"
#include "ros/micro_ros/type_support.hpp"

#include <micro_ros_utilities/type_utilities.h>
#include <rcl/types.h>
#include <span>

namespace br_messages {

namespace msg {
class DisplacementOrder : public br_messages__msg__DisplacementOrder {
  public:
    DisplacementOrder() {
        micro_ros_utilities_memory_conf_t conf = generateMemoryConf();
        RCCHECK_HARD(micro_ros_utilities_create_message_memory(type_support(), this, conf) ? RCL_RET_OK : RCL_RET_ERROR);
    }

    DisplacementOrder(const DisplacementOrder &) = delete;
    DisplacementOrder &operator=(const DisplacementOrder &) = delete;

    DisplacementOrder(DisplacementOrder &&other) { *this = std::move(other); }
    DisplacementOrder &operator=(DisplacementOrder &&other) {
        path = other.path;
        theta = other.theta;
        kind = other.kind;

        other.path.capacity = 0;
        other.path.size = 0;
        other.path.data = nullptr;

        return *this;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, DisplacementOrder); }
    static constexpr std::size_t MAX_PATH_LEN = 20;

  private:
    static constexpr micro_ros_utilities_memory_conf_t generateMemoryConf() {
        micro_ros_utilities_memory_conf_t conf = {0};
        conf.max_ros2_type_sequence_capacity = MAX_PATH_LEN;
        return conf;
    }
};
} // namespace msg

namespace __detail {
class __PathView {
    const br_messages__msg__Point *m_begin;
    std::size_t m_size;

  public:
    class Iterator {
        const br_messages__msg__Point *m_ptr;
        friend class __PathView;

        Iterator(const br_messages__msg__Point *ptr) : m_ptr(ptr) {}

      public:
        msg::Point operator*() const { return msg::Point(*m_ptr); }
        Iterator &operator++() {
            ++m_ptr;
            return *this;
        }
        Iterator operator++(int) {
            Iterator cpy = *this;
            ++(*this);
            return cpy;
        }
        bool operator==(const Iterator &other) const { return m_ptr == other.m_ptr; }
        bool operator!=(const Iterator &other) const { return m_ptr != other.m_ptr; }
    };

    using iterator = Iterator;

    __PathView(const br_messages__msg__Point__Sequence &path) : m_begin(path.data), m_size(path.size) {}
    Iterator begin() const { return Iterator(m_begin); }
    Iterator end() const { return Iterator(m_begin + m_size); }
    std::size_t size() const { return m_size; }
    bool empty() const { return m_size == 0; }
    msg::Point operator[](std::size_t i) const { return msg::Point(m_begin[i]); }
};
} // namespace __detail

} // namespace br_messages

#endif