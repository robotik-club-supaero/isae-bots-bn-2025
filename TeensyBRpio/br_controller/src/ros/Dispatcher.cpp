#include "ros/Dispatcher.hpp"
#include "controller/DisplacementKind.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/PathTrajectory.hpp"
#include "trajectories/PolygonalTrajectory.hpp"

constexpr uint8_t FLAG_REVERSE = 0b0001;
constexpr uint8_t FLAG_FINAL_ORIENTATION = 0b0010;
constexpr uint8_t FLAG_ALLOW_CURVE = 0b0100;

constexpr double_t CONTROL_MAX_SPEED = 255.;

using namespace ros2;

template <typename TManager>
inline void handleRotate(TManager &manager, const Angle &orientation) {
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) {
        controller.template startRotation<SetHeadingProfile>(robotPosition.theta, orientation);
    });
}

template <typename TManager>
inline void handleGoTo(TManager &manager, const br_messages::msg::DisplacementOrder &order) {
    DisplacementKind kind = ((order.kind & FLAG_REVERSE) == 0) ? FORWARD : REVERSE;
    std::optional<Angle> finalOrientation = ((order.kind & FLAG_FINAL_ORIENTATION) != 0) ? std::make_optional(order.theta) : std::nullopt;
    bool allowCurve = (order.kind & FLAG_ALLOW_CURVE) != 0;

    auto path_view = br_messages::getPathView(order);
    if (path_view.empty()) {
        if (finalOrientation) {
            handleRotate(manager, *finalOrientation);
        }
        return;
    }

    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) {
        // We cancel the previous order to free the memory used by the previous trajectory (if any).
        // This reduces fragmentation and the probability of allocation failure.
        controller.brakeToStop();

        SmallDeque<Point2D<Meter>> path;
        if (!path.reserve(path_view.size() + 1)) {
            log(ERROR, "Could not initialize trajectory because of memory exhaustion");
            // TODO callback
            return;
        }
        for (const Point2D<Meter> &point : path_view) {
            std::ignore = path.push_back(point);
        }
        controller.startPath(kind, allowCurve, robotPosition, finalOrientation, std::move(path));
    });
}

template <typename TManager>
inline void handleStop(TManager &manager, const std_msgs::msg::Empty &_msg) {
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); });
}

template <typename TManager>
inline void handleCommand(TManager &manager, const br_messages::msg::Command &command) {
    Speeds speeds = br_messages::command_cast(command) / CONTROL_MAX_SPEED;
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) { //
        speeds.linear *= controller.getMaxSpeeds().linear;
        speeds.angular *= controller.getMaxSpeeds().angular;

        controller.setSetpointSpeed(speeds /*, enforceMaxSpeeds = true */);
    });
}

template <typename TManager>
inline void handleIdle(TManager &manager, const std_msgs::msg::Bool &msg) {
    manager.setActive(msg.data);
}

template <typename TManager>
inline void handleResetPosition(TManager &manager, const br_messages::msg::Position &position) {
    manager.resetPosition(br_messages::position_cast<Meter>(position));
}

template <typename TManager>
inline void handleSetGains(TManager &manager, const br_messages::msg::GainsPid &gains) {
    auto pid = manager.getController().getErrorConverter();
    manager.getController().setErrorConverter(
        {gains.kp, gains.ti, gains.td, pid.filter(), pid.saturation(), pid.integralSaturation(), pid.derivativeSaturation()});
}

template <typename TManager>
inline void handleSetSpeed(TManager &manager, const std_msgs::msg::Int16 &speedFactor) {
    Speeds maxSpeeds = manager.getController().getMaxSpeeds();
    manager.getController().setMaxSpeeds(maxSpeeds * clamp(static_cast<double_t>(speedFactor.data), 1., 100.) / 100.,
                                         /* persist = */ false);
}

#define ROS_CALLBACK(fn) [](TManager &manager, const auto &msg) { fn(manager, msg); }

template <typename TManager>
Dispatcher<TManager>::Dispatcher(Node &node, std::weak_ptr<TManager> manager)
    : m_subGoTo(node, "/br/goTo", manager, ROS_CALLBACK(handleGoTo)),          //
      m_subStop(node, "/br/stop", manager, ROS_CALLBACK(handleStop)),          //
      m_subCommand(node, "/br/command", manager, ROS_CALLBACK(handleCommand)), //
      m_subIdle(node, "/br/idle", manager, ROS_CALLBACK(handleIdle)),          //
      m_subReset(node, "/br/resetPosition", manager, ROS_CALLBACK(handleResetPosition)),
      m_subGains(node, "/br/gains", manager, ROS_CALLBACK(handleSetGains)), //
      m_subSpeed(node, "/br/setSpeed", manager, ROS_CALLBACK(handleSetSpeed)) {}

// Explicit instantiation of the Dispatcher class
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/manager.hpp"
template class Dispatcher<manager_t>;