#ifndef _CONTROLLER_STATE_HPP_
#define _CONTROLLER_STATE_HPP_

#include "controller/StateResult.hpp"
#include "geometry/Position2D.hpp"

namespace controller {

/// The current status of the controller.
class ControllerStatus {
  public:
    enum Flag : uint8_t {
        /// Is stopping (not set when already stopped)?
        STOPPING = 0b00001,
        /// Is a trajectory ongoing or pending?
        TRAJECTORY = 0b00010,
        /// Is moving straight?
        MOVING = 0b00100,
        /// Is rotating?
        ROTATING = 0b01000,
        /// Speed control instead of position control. The states that rely on speed control should set this flag, but this is only a hint.
        /// The actual control kind only depends on the return value of ControllerState::update.
        SPEED_CONTROL = 0b10000,

        /// When used in combination with MOVING, means REVERSE.
        /// Otherwise, has no special meaning (used to distinguish states).
        EXTRA_1 = 0b100000
    };

    constexpr ControllerStatus() = default;
    constexpr ControllerStatus(uint8_t value) : m_value(value) {}
    constexpr operator uint8_t() const { return m_value; }

    static constexpr uint8_t REVERSE = EXTRA_1 | MOVING;

    static constexpr uint8_t Invalid = ~(SPEED_CONTROL | ROTATING | MOVING | TRAJECTORY | STOPPING);
    /// The robot is standing still close to its rest point
    static constexpr uint8_t Still = 0;
    static constexpr uint8_t Braking = STOPPING | SPEED_CONTROL;
    /// The robot is rotating in the direction of the trajectory it has to follow (required due to being unicycle)
    static constexpr uint8_t InitialRotation = ROTATING | TRAJECTORY;
    /// The robot is following a trajectory
    static constexpr uint8_t Forward = MOVING | TRAJECTORY;
    /// The robot is following a reverse trajectory
    static constexpr uint8_t Reversing = REVERSE | TRAJECTORY;
    /// The robot is rotating in the final requested direction (the final rotation is not considered to be part of the trajectory)
    static constexpr uint8_t FinalRotation = ROTATING;
    static constexpr uint8_t ManualControl = SPEED_CONTROL | ROTATING | MOVING;

  private:
    uint8_t m_value;
};

} // namespace controller

#endif