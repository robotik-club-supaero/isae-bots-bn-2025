#ifndef _ROS_HPP_
#define _ROS_HPP_

#include "configuration.hpp"

#include "logging.hpp"
#include "manager/ControllerManager.hpp"
#include "ros/Callbacks.hpp"
#include "ros/Dispatcher.hpp"
#include "ros/ros_impl.hpp"
#include "specializations/controller.hpp"

#include <memory>
#include <optional>

/**
 * ROS2 node to receive displacement orders and send callbacks.
 *
 * @tparam TActuators,TFeedback must be the same as the manager
 */
template <Actuators TActuators, PositionFeedback TFeedback>
class ROS : public ros2::Node {
  public:
    using manager_t = manager::ControllerManager<TActuators, controller_t, TFeedback>;

    template <typename T>
    using subscription_t = ros2::Subscription<T>;
    template <typename T>
    using publisher_t = ros2::Publisher<T>;

    /**
     * @param sendPositionInterval,logInterval In microseconds
     */
    ROS(duration_t sendPositionInterval, duration_t logInterval);
    ROS(const ROS<TActuators, TFeedback> &) = delete;

    ROS() : ROS(SEND_POSITION_INTERVAL * 1000, ROS_LOG_INTERVAL * 1000) {}

    /// The manager is attached later to enable logging of early errors during creation of the manager or its dependencies.
    /// The subscribtions are not created until this method is called. This method must be called exactly once.
    void attachManager(std::shared_ptr<manager_t> manager);

    template <typename... Args>
    void attachManager(Args &&...args) {
        attachManager(std::make_shared<manager_t>(std::forward<Args>(args)...));
    }

    /// Spins the ROS node and call "loop" on the attached manager. It is an error to call this function before a manager is attached.
    void loop();
    void sendLog(LogSeverity severity, const char *message);

  private:
    std::shared_ptr<manager_t> m_manager;

    duration_t m_sendInterval;
    duration_t m_logInterval;

    instant_t m_lastSend;
    instant_t m_lastLog;
    bool m_wasActive;

    br_messages::msg::LogEntry m_log;

    /* SUBSCRIBERS */
    std::optional<Dispatcher<manager_t>> m_dispatcher;

    /* PUBLISHERS */

    /// /br/currentPosition
    publisher_t<br_messages::msg::Position> m_pubPositionFeedback;
    /// /br/callbacks
    publisher_t<std_msgs::msg::Int16> m_pubHN;
    /// /br/logTotaleArray
    publisher_t<br_messages::msg::LogEntry> m_pubLog;
    /// /br/odosCount
    publisher_t<br_messages::msg::OdosCount> m_pubOdosTicks;

    void sendCallback(AsservCallback callback);
};

#endif