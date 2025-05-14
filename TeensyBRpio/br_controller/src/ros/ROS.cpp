#include "ros/ROS.hpp"
#include "defines/math.hpp"
#include "logging.hpp"
#include "ros/Callbacks.hpp"

using UpdateResultCode = controller::UpdateResultCode;

#define TEMPLATE template <Actuators TActuators, PositionFeedback TFeedback>
#define _ROS ROS<TActuators, TFeedback>

template <typename T>
constexpr bool hasOdos = requires(const T &t) {
    t.getLeftOdoCount();
    t.getRightOdoCount();
};

template <typename T>
constexpr bool hasTwoWheels = requires(const T &t) {
    t.getLastLeftSpeed();
    t.getLastRightSpeed();
};

TEMPLATE
_ROS::ROS(duration_t sendPositionInterval, duration_t logInterval)
    : ros2::Node("base_roulante"), m_sendInterval(sendPositionInterval), m_logInterval(logInterval), m_lastSend(0), m_lastLog(0), m_wasActive(false),
      m_dispatcher(), m_pubPositionFeedback(this->template createPublisher<br_messages::msg::Position>("/br/currentPosition")),
      m_pubHN(this->template createPublisher<std_msgs::msg::Int16>("/br/callbacks", /* reliability = */ ReliableOnly)), //
      m_pubLog(this->template createPublisher<br_messages::msg::LogEntry>("/br/logTotaleArray")),
      m_pubOdosTicks(this->template createPublisher<br_messages::msg::OdosCount>("/br/odosCount")) {}

TEMPLATE
void _ROS::attachManager(std::shared_ptr<manager_t> manager) {
    if (m_manager) {
        log(ERROR, "A manager is already attached to this ROS instance");
        return;
    }
    m_manager = std::move(manager);

    m_dispatcher.emplace(*this, std::weak_ptr(m_manager));
}

TEMPLATE
void _ROS::loop() {
    this->spin_once();

    if (!m_manager) {
        log(ERROR, "Called ROS::loop() before attaching a manager");
        return;
    }

    while (m_manager->update()) {
        if (!m_wasActive && m_manager->getStatus() == manager::Active) {
            m_wasActive = true;
            sendCallback(OK_READY);
        } else if (m_wasActive && m_manager->getStatus() == manager::Idle) {
            m_wasActive = false;
            sendCallback(OK_IDLE);
        }

        UpdateResultCode event = m_manager->getController().getLastEvent();
        if (event & UpdateResultCode::ROTATION_COMPLETE) {
            sendCallback(OK_TURN);
        }
        if (event & UpdateResultCode::TRAJECTORY_COMPLETE) {
            if ((event & UpdateResultCode::WAS_REVERSE) && (event & UpdateResultCode::TERMINAL)) {
                sendCallback(OK_REVERSE);
            }
            sendCallback(OK_POS);
        }
        if ((event & UpdateResultCode::TERMINAL) && event != UpdateResultCode::STOPPED) {
            sendCallback(OK_ORDER);
        }
    }

    duration_t now = micros();
    if (getDurationMicros(m_lastSend, now) > m_sendInterval) {
        m_lastSend = now;
        m_pubPositionFeedback.publish(br_messages::position_cast(m_manager->getPositionFeedback().getRobotPosition().toMillimeters()));

        if constexpr (hasOdos<TFeedback>) {
            const TFeedback &feedback = m_manager->getPositionFeedback();
            m_pubOdosTicks.publish(br_messages::make_odos_count(feedback.getLeftOdoCount(), feedback.getRightOdoCount()));
        }
    }

#ifdef _BR_DEBUG
    if (getDurationMicros(m_lastLog, now) > m_logInterval) {
        m_lastLog = now;

        m_log.time = now;

        m_log.robot_pos = br_messages::position_cast(m_manager->getPositionFeedback().getRobotPosition());
        m_log.setpoint_pos = br_messages::position_cast(m_manager->getController().getSetpoint());
        m_log.goal_point_pos = br_messages::point_cast(m_manager->getController().getGoalPoint());
        m_log.goal_point_speed = br_messages::point_cast(m_manager->getController().getGoalPointSpeed());
        m_log.asserv_error = br_messages::point_cast(m_manager->getController().getErrorConverter().lastError());
        m_log.command = br_messages::command_cast(m_manager->getController().getLastCommand());

        if constexpr (hasTwoWheels<TActuators>) {
            m_log.commande_motor_r = m_manager->getActuators().getLastRightSpeed();
            m_log.commande_motor_l = m_manager->getActuators().getLastLeftSpeed();
        }

        m_log.manager_state = m_manager->getStatus();
        m_log.controller_state = m_manager->getController().getStatus();

        m_pubLog.publish(m_log);
    }
#endif
}

TEMPLATE
void _ROS::sendLog(LogSeverity severity, const char *message) {
    ros2::Node::sendLog(severity, message);
}

TEMPLATE
void _ROS::sendCallback(AsservCallback callback) {
    m_pubHN.publish(std_msgs::make_message(static_cast<int16_t>(callback)));
}

// Explicit instantiation of the ROS node
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/ros.hpp"
template class ROS<actuators_t, feedback_t>;