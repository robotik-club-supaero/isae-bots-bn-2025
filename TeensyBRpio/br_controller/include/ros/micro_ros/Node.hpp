#ifndef _ROS_IMPL_NODE_HPP_
#define _ROS_IMPL_NODE_HPP_

#include "Clock.hpp"
#include "logging.hpp"

#include "ros/micro_ros/Publisher.hpp"
#include "ros/micro_ros/Subscriber.hpp"
#include "ros/micro_ros/macros.hpp"

#include <rcl/rcl.h>
#include <rcl_interfaces/msg/log.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <functional>
#include <memory>
#include <optional>

namespace ros2 {

namespace rcl_interfaces {
namespace msg {
class Log : public rcl_interfaces__msg__Log {
  public:
    using rcl_interfaces__msg__Log::rcl_interfaces__msg__Log;
    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log); }
};
} // namespace msg
} // namespace rcl_interfaces

class Node {

  public:
    Node(const char *name) : m_msgLog(), m_logger() {
        RCCHECK_HARD(rclc_support_init(m_support.get(), 0, NULL, &m_allocator));
        RCCHECK_HARD(rclc_node_init_default(m_node.get(), name, "", m_support.get()));
        RCCHECK_HARD(rclc_executor_init(m_executor.get(), &m_support->context, 7, &m_allocator));

        rcl_interfaces__msg__Log__init(&m_msgLog);
        rosidl_runtime_c__String__assign(&m_msgLog.name, name);

        m_logger.emplace(createPublisher<rcl_interfaces::msg::Log>("rosout", /* reliability = */ ReliableOnly));
    }

    void spin_once() { RCCHECK_HARD(rclc_executor_spin_some(m_executor.get(), 0)); }

    template <typename T>
    Subscription<T> createSubscription(const char *topic, std::function<void(const T &)> callback) {
        return Subscription<T>(m_node.get(), m_executor.get(), topic, callback);
    }

    template <typename T>
    Publisher<T> createPublisher(const char *topic, QosReliability reliability = AllowBestEffort) {
        return Publisher<T>(m_node.get(), topic, reliability);
    }

    void sendLog(LogSeverity severity, const char *message) {
        switch (severity) {
            case INFO:
                m_msgLog.level = rcl_interfaces__msg__Log__INFO;
                break;
            case WARN:
                m_msgLog.level = rcl_interfaces__msg__Log__WARN;
                break;
            case ERROR:
                m_msgLog.level = rcl_interfaces__msg__Log__ERROR;
                break;
            case FATAL:
                m_msgLog.level = rcl_interfaces__msg__Log__FATAL;
                break;
            case DEBUG:
                m_msgLog.level = rcl_interfaces__msg__Log__DEBUG;
                break;
            default:
                sendLog(ERROR, "Unknown log type; defaulting to INFO");
                m_msgLog.level = rcl_interfaces__msg__Log__INFO;
                break;
        }

        rosidl_runtime_c__String__assign(&m_msgLog.msg, message);

        instant_t stamp = micros();
        m_msgLog.stamp.sec = static_cast<int32_t>(stamp / 1000000);
        m_msgLog.stamp.nanosec = static_cast<int32_t>((stamp % 1000000) * 1000);

        if (m_logger) {
            m_logger->publish(m_msgLog);
        }
    }

  private:
    rcl_allocator_t m_allocator = rcl_get_default_allocator();
    std::unique_ptr<rclc_support_t> m_support = std::make_unique<rclc_support_t>();
    std::unique_ptr<rclc_executor_t> m_executor = std::make_unique<rclc_executor_t>();
    std::unique_ptr<rcl_node_t> m_node = std::make_unique<rcl_node_t>();

    rcl_interfaces::msg::Log m_msgLog;
    // We use our own publisher to /rosout instead of the logging macros because micro_ros does not publish logs automatically
    std::optional<Publisher<rcl_interfaces::msg::Log>> m_logger;
};
} // namespace ros2

#endif