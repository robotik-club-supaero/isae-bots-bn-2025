#include "ros/ROS.hpp"
#include "defines/math.hpp"
#include "logging.hpp"
#include "ros/Callbacks.hpp"

using UpdateResultCode = controller::UpdateResultCode;

#define TEMPLATE template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
#define _ROS ROS<TActuators, TFeedback, TClock>

using namespace ros_impl::messages;

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
_ROS::ROS(TClock clock, duration_t sendPositionInterval, duration_t logInterval)
    : ros_impl::node_t("base_roulante"), m_clock(std::move(clock)), m_sendInterval(sendPositionInterval), m_logInterval(logInterval), m_lastSend(0),
      m_lastLog(0), m_wasActive(false), m_dispatcher(), m_pubPositionFeedback(this->template createPublisher<position_t>("/br/currentPosition")),
      m_pubHN(this->template createPublisher<msg_int16_t>("/br/callbacks", /* reliability = */ ReliableOnly)), //
      m_pubLog(this->template createPublisher<log_entry_t>("/br/logTotaleArray")),
      m_pubOdosTicks(this->template createPublisher<odos_count_t>("/br/odosCount")) {}

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
            m_pubHN.publish(OK_READY);
        } else if (m_wasActive && m_manager->getStatus() == manager::Idle) {
            m_wasActive = false;
            m_pubHN.publish(OK_IDLE);
        }

        UpdateResultCode event = m_manager->getController().getLastEvent();
        if (event & UpdateResultCode::ROTATION_COMPLETE) {
            m_pubHN.publish(OK_TURN);
        }
        if (event & UpdateResultCode::TRAJECTORY_COMPLETE) {
            if ((event & UpdateResultCode::WAS_REVERSE) && (event & UpdateResultCode::TERMINAL)) {
                m_pubHN.publish(OK_REVERSE);
            }
            m_pubHN.publish(OK_POS);
        }
        if ((event & UpdateResultCode::TERMINAL) && event != UpdateResultCode::STOPPED) {
            m_pubHN.publish(OK_ORDER);
        }
    }

    duration_t now = m_clock.micros();
    if (getDurationMicros(m_lastSend, now) > m_sendInterval) {
        m_lastSend = now;
        m_pubPositionFeedback.publish(m_manager->getPositionFeedback().getRobotPosition().toMillimeters());

        if constexpr (hasOdos<TFeedback>) {
            const TFeedback &feedback = m_manager->getPositionFeedback();
            m_pubOdosTicks.publish(odos_count_t{feedback.getLeftOdoCount(), feedback.getRightOdoCount()});
        }
    }

#ifdef _BR_DEBUG
    if (getDurationMicros(m_lastLog, now) > m_logInterval) {
        m_lastLog = now;

        m_log.time = now;

        m_log.robot_pos = message_cast<position_t>(m_manager->getPositionFeedback().getRobotPosition());
        m_log.setpoint_pos = message_cast<position_t>(m_manager->getController().getSetpoint());
        m_log.goal_point_pos = message_cast<point_t>(m_manager->getController().getGoalPoint());
        m_log.goal_point_speed = message_cast<point_t>(m_manager->getController().getGoalPointSpeed());
        m_log.asserv_error = message_cast<point_t>(m_manager->getController().getErrorConverter().lastError());
        m_log.command = message_cast<command_t>(m_manager->getController().getLastCommand());

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
    ros_impl::node_t::sendLog(severity, message);
}

// Explicit instantiation of the ROS node
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/ros.hpp"
template class ROS<actuators_t, feedback_t, _clock_t>;