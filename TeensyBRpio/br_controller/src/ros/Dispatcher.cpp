#include "ros/Dispatcher.hpp"
#include "controller/DisplacementKind.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/PathTrajectory.hpp"
#include "trajectories/PolygonalTrajectory.hpp"

constexpr uint8_t FLAG_REVERSE = 0b0001;
constexpr uint8_t FLAG_FINAL_ORIENTATION = 0b0010;
constexpr uint8_t FLAG_ALLOW_CURVE = 0b0100;

constexpr double_t CONTROL_MAX_SPEED = 255.;

using namespace ros_impl;
using namespace ros_impl::messages;

template <typename TManager>
inline void handleRotate(TManager &manager, const Angle &orientation) {
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) {
        controller.template startRotation<SetHeadingProfile>(robotPosition.theta, orientation);
    });
}

template <typename TManager>
inline void handleGoTo(TManager &manager, const displacement_order_t &order) {
    DisplacementKind kind = ((order.kind & FLAG_REVERSE) == 0) ? FORWARD : REVERSE;
    std::optional<Angle> finalOrientation = ((order.kind & FLAG_FINAL_ORIENTATION) != 0) ? std::make_optional(order.theta) : std::nullopt;
    bool allowCurve = (order.kind & FLAG_ALLOW_CURVE) != 0;

    auto pathRaw = message_cast<std::span<const point_t>>(order.path);
    if (pathRaw.empty()) {
        if (finalOrientation) {
            handleRotate(manager, *finalOrientation);
        }
        return;
    }
    std::vector<Point2D<Meter>> path;
    path.reserve(pathRaw.size() + 1);

    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) {
        path.push_back(robotPosition);
        for (point_t point : pathRaw) {
            path.push_back(message_cast<Point2D<Millimeter>>(point).toMeters());
        }

        Angle initialDirection = robotPosition.theta + kind.getAlignmentOffset();
        std::unique_ptr<Trajectory> trajectory;
        if (allowCurve) {
            trajectory = std::make_unique<PathTrajectory>(initialDirection, std::move(path));
        } else {
            trajectory = std::make_unique<PolygonalTrajectory>(std::move(path));
        }
        controller.startTrajectory(kind, std::move(trajectory), finalOrientation);
    });
}

template <typename TManager>
inline void handleStop(TManager &manager, const empty_t &_msg) {
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); });
}

template <typename TManager>
inline void handleCommand(TManager &manager, const command_t &command) {
    Speeds speeds = message_cast<Speeds>(command) / CONTROL_MAX_SPEED;
    manager.sendOrder([&](TManager::controller_t &controller, Position2D<Meter> robotPosition) { //
        speeds.linear *= controller.getMaxSpeeds().linear;
        speeds.angular *= controller.getMaxSpeeds().angular;

        controller.setSetpointSpeed(speeds /*, enforceMaxSpeeds = true */);
    });
}

template <typename TManager>
inline void handleIdle(TManager &manager, const bool_t &msg) {
    manager.setActive(message_cast<const bool &>(msg));
}

template <typename TManager>
inline void handleResetPosition(TManager &manager, const position_t &position) {
    manager.resetPosition(message_cast<Position2D<Millimeter>>(position).toMeters());
}

template <typename TManager>
inline void handleSetGains(TManager &manager, const gains_t &gains) {
    auto pid = manager.getController().getErrorConverter();
    manager.getController().setErrorConverter(
        {gains.kp, gains.ti, gains.td, pid.filter(), pid.saturation(), pid.integralSaturation(), pid.derivativeSaturation()});
}

template <typename TManager>
inline void handleSetSpeed(TManager &manager, const msg_int16_t &speedFactor) {
    Speeds maxSpeeds = manager.getController().getMaxSpeeds();
    manager.getController().setMaxSpeeds(maxSpeeds * clamp(static_cast<double_t>(message_cast<const int16_t &>(speedFactor)), 1., 100.) / 100.,
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