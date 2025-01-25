#ifndef _CONTROLLER_STATE_RESULT_HPP_
#define _CONTROLLER_STATE_RESULT_HPP_

#include "geometry/Angle.hpp"
#include "geometry/Position2D.hpp"

#include <cstdint>
#include <optional>
#include <variant>

namespace controller {

/// Requests to move the robot to the requested position
class PositionControl {
  public:
    /// Requests that the setpoint do not move
    PositionControl() : setpoint(), relative(true) {}
    PositionControl(Position2D<Meter> setpoint, bool relative = false) : setpoint(setpoint), relative(relative) {}

    /// The new position of the setpoint
    Position2D<Meter> setpoint;
    /// Whether the position is absolute or relative to the previous setpoint
    bool relative;
};
/// Requests to change the robot's heading without moving its center
class OrientationControl {
  public:
    OrientationControl(Angle orientation, bool relative = false) : orientation(orientation), relative(relative) {}

    Angle orientation;
    /// Whether the new orientation is absolute or relative to the previous setpoint
    bool relative;
};
/// Requests to move the robot with the requested speed
class SpeedControl {
  public:
    SpeedControl(Speeds speeds) : speeds(speeds) {}
    Speeds speeds;
};

class TrajectoryComplete {
  public:
    TrajectoryComplete(std::optional<Angle> finalOrientation = std::nullopt) : finalOrientation(finalOrientation) {}

    std::optional<Angle> finalOrientation;
};
class RotationComplete {};
class BrakingComplete {};

using StateUpdateResult = std::variant<PositionControl, OrientationControl, SpeedControl, TrajectoryComplete, RotationComplete, BrakingComplete>;

/// Code returned by the controller to help implement ROS's callbacks. 
class UpdateResultCode {
  public:
    enum Flag : uint8_t {
        /// End of order (end of displacement without final orientation or end of final rotation)
        TERMINAL = 1,
        TRAJECTORY_COMPLETE = 2,
        ROTATION_COMPLETE = 4,
        WAS_REVERSE = 8,
    };
    constexpr UpdateResultCode() = default;
    constexpr UpdateResultCode(uint8_t value) : m_value(value) {}
    constexpr operator uint8_t() const { return m_value; }

    static constexpr uint8_t STOPPED = TERMINAL;

    static constexpr uint8_t INITIAL_ROTATION_COMPLETE = ROTATION_COMPLETE;
    static constexpr uint8_t FINAL_ROTATION_COMPLETE = ROTATION_COMPLETE | TERMINAL;

    static constexpr uint8_t FORWARD_TRAJECTORY_COMPLETE = TRAJECTORY_COMPLETE;
    static constexpr uint8_t REVERSE_TRAJECTORY_COMPLETE = TRAJECTORY_COMPLETE | WAS_REVERSE;

    static constexpr uint8_t ARRIVED_FORWARD = FORWARD_TRAJECTORY_COMPLETE | TERMINAL;
    static constexpr uint8_t ARRIVED_REVERSE = REVERSE_TRAJECTORY_COMPLETE | TERMINAL;

  private:
    uint8_t m_value;
};

} // namespace controller

#endif