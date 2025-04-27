#include "controller/UnicycleController.hpp"
#include "controller/states/StateBraking.hpp"
#include "controller/states/StateFullTrajectory.hpp"
#include "controller/states/StateManualControl.hpp"
#include "controller/states/StateRotation.hpp"
#include "controller/states/StateStandStill.hpp"

#include "defines/func.hpp"
#include "defines/math.hpp"
#include "logging.hpp"
#include "rotations/SetHeadingProfile.hpp"

#include <numbers>

namespace controller {

template <ErrorConverter TConverter>
UnicycleController<TConverter>::UnicycleController(Vector2D<Meter> trackingOffset, TConverter converter, Accelerations brakeAccelerations,
                                                   Speeds maxSpeeds, Accelerations maxAccelerations, double_t speedThreshold)
    : m_offset(trackingOffset), m_brakeAccelerations(brakeAccelerations), m_maxSpeeds(maxSpeeds), m_maxAccelerations(maxAccelerations),
      m_speedThreshold(speedThreshold), m_reversing(), m_speedControl(), m_setpoint(Position2D<Meter>()), m_goalPointSpeed(), m_actualSpeed(),
      m_trackingPointSpeed(1, 0.2), m_converter(std::move(converter)), m_event(), m_suspendedState(), m_lastCommand() {
    setCurrentState<StateStandStill>(Position2D<Meter>(0, 0, 0), /* advertize = */ false);
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::updateCommand(double_t interval, Position2D<Meter> robotPosition) {
    m_event = UpdateResultCode();
    m_actualSpeed.update(robotPosition, interval);

    Vector2D<Meter> trackingPoint = applyOffset(robotPosition);
    m_trackingPointSpeed.update(trackingPoint, interval);

    if (m_speedControl) {
        softReset(robotPosition);
    }

    setpoint_t newSetpoint = computeSetpoint(interval, robotPosition);
    m_speedControl = std::holds_alternative<Speeds>(newSetpoint);

    Speeds command;

    // TODO this may not be the only condition to suspend displacement (how to detect severe wind up?)
    if (!m_speedControl && abs((m_setpoint - robotPosition).theta) > Angle::Pi / 3) {
        m_suspendedState = replaceCurrentState<StateBraking>(getEstimatedRelativeRobotSpeed(), m_brakeAccelerations);
        command = m_lastCommand;
        log(WARN, "Recovering from wrong robot orientation");
    } else {
        command = computeCommand(interval, robotPosition, newSetpoint);
    }

    m_lastCommand = command;
    return command;
}

// Private
template <ErrorConverter TConverter>
setpoint_t UnicycleController<TConverter>::computeSetpoint(double_t interval, Position2D<Meter> robotPosition) {
    StateUpdateResult result = this->getCurrentState().update(interval);
    return std::visit<setpoint_t>( //
        overload{
            [&](PositionControl &r) {
                if (r.relative) {
                    return m_setpoint.relativeOffset(r.setpoint.x, r.setpoint.y, r.setpoint.theta);
                } else {
                    return r.setpoint;
                }
            }, //
            [&](OrientationControl &r) {
                Position2D<Meter> currentSetpoint = m_setpoint;
                if (r.relative) {
                    currentSetpoint.theta += r.orientation;
                } else {
                    currentSetpoint.theta = r.orientation;
                }
                return currentSetpoint;
            }, //
            [&](SpeedControl &r) { return r.speeds; },
            [&](TrajectoryComplete &r) {
                Position2D<Meter> currentSetpoint = m_setpoint;
                if (!isMoving()) {
                    bool was_reverse = (getStatus() & REVERSE) != 0;
                    if (r.finalOrientation) {
                        m_event = UpdateResultCode::TRAJECTORY_COMPLETE;
                        startRotation<SetHeadingProfile>(currentSetpoint.theta, *r.finalOrientation);
                    } else {
                        m_event = UpdateResultCode::ARRIVED_FORWARD;
                        setCurrentState<StateStandStill>(currentSetpoint);
                    }
                    if (was_reverse) {
                        m_event = m_event | UpdateResultCode::WAS_REVERSE;
                    }
                }
                return currentSetpoint;
            },
            [&](RotationComplete &r) {
                Position2D<Meter> currentSetpoint = m_setpoint;
                m_event = UpdateResultCode::FINAL_ROTATION_COMPLETE;
                setCurrentState<StateStandStill>(currentSetpoint);
                return currentSetpoint;
            },
            [&](BrakingComplete &r) {
                if (!isMoving()) {
                    if (m_suspendedState && m_suspendedState->resumeState(robotPosition)) {
                        replaceCurrentState(std::move(m_suspendedState));
                    } else {
                        setCurrentState<StateStandStill>(robotPosition);
                    }
                    return setpoint_t(robotPosition);
                } else {
                    return setpoint_t(Speeds(0, 0));
                }
            } //
        },
        result);
}

// Private
template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::computeCommand(double_t interval, Position2D<Meter> robotPosition, setpoint_t setpoint) {
    return std::visit( //
        overload{
            [&](Position2D<Meter> &setpoint) {
                // The robot has a preference for going forward due to the tracking point being in front of it.
                // This stabilizes the control during reverse gears.
                bool reversing = setpoint.makeRelative(setpoint - m_setpoint).x < 0;
                if (reversing != m_reversing) {
                    m_reversing = reversing;
                    m_converter.reset();
                    m_goalPointSpeed.reset(applyOffset(reversing ? m_setpoint.flip() : m_setpoint));
                    if (reversing) {
                        log(DEBUG, "Detected reverse displacement.");
                    } else {
                        log(DEBUG, "End of reverse displacement.");
                    }
                }
                m_setpoint = setpoint;

                if (reversing) {
                    setpoint = setpoint.flip();
                    robotPosition = robotPosition.flip();
                }

                // Apply tracking offset
                Vector2D<Meter> trackingPoint = reversing ? applyOffset(robotPosition) : m_trackingPointSpeed.getLastInput();
                Vector2D<Meter> goalPoint = applyOffset(setpoint);
                m_goalPointSpeed.update(goalPoint, interval);

                // Compute error
                m_converter.update(goalPoint - trackingPoint, interval);
                Vector2D<Meter> convertedError = m_goalPointSpeed.value() + m_converter.value();

                // Compute command from error
                double_t alpha = m_offset.x;
                double_t beta = m_offset.y;
                double_t theta = robotPosition.theta;
                double_t cos_theta = std::cos(theta);
                double_t sin_theta = std::sin(theta);

                double_t cmd_v =
                    ((alpha * cos_theta - beta * sin_theta) * convertedError.x + (alpha * sin_theta + beta * cos_theta) * convertedError.y) / alpha;
                double_t cmd_omega = (-sin_theta * convertedError.x + cos_theta * convertedError.y) / alpha;

                if (reversing) {
                    cmd_v = -cmd_v;
                }

                return Speeds(cmd_v, cmd_omega);
            },
            [](Speeds speeds) { return speeds; } //
        },
        setpoint);
}

// Orders

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setSetpoint(Position2D<Meter> setpoint) {
    m_setpoint = setpoint;
    setCurrentState<StateStandStill>(setpoint);
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setSetpointSpeed(Speeds speeds, bool enforceMaxSpeeds, bool enforceMaxAccelerations) {
    if (getStatus() != ControllerStatus::ManualControl) {
        startDisplacement<StateManualControl>(m_maxAccelerations);
    }
    if (enforceMaxSpeeds) {
        speeds.linear = clamp(speeds.linear, -m_maxSpeeds.linear, m_maxSpeeds.linear);
        speeds.angular = clamp(speeds.angular, -m_maxSpeeds.angular, m_maxSpeeds.angular);
    }

    if (m_suspendedState) {
        m_suspendedState->notify(ManualSpeedCommand(speeds, enforceMaxAccelerations));
    } else {
        getCurrentState().notify(ManualSpeedCommand(speeds, enforceMaxAccelerations));
    }
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::brakeToStop() {
    m_suspendedState.reset();
    if ((getStatus() & (ControllerStatus::ROTATING | ControllerStatus::MOVING | ControllerStatus::TRAJECTORY)) != 0) {
        // TODO should we use getEstimatedRelativeRobotSpeed() or m_lastCommand as the initial speed to give to StateBraking?
        setCurrentState<StateBraking>(getEstimatedRelativeRobotSpeed(), m_brakeAccelerations);
    }
}

template <ErrorConverter TConverter>
bool UnicycleController<TConverter>::isMoving() const {
    return m_goalPointSpeed.value().norm() > m_speedThreshold || m_trackingPointSpeed.value().norm() > m_speedThreshold;
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::startTrajectory(DisplacementKind kind, std::unique_ptr<Trajectory> trajectory,
                                                     std::optional<Angle> finalOrientation) {
    softReset({trajectory->getCurrentPosition(), m_setpoint.theta});
    startDisplacement<StateFullTrajectory>(kind, std::move(trajectory), m_setpoint.theta, m_maxSpeeds, m_maxAccelerations, finalOrientation);
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::startRotation(std::unique_ptr<OrientationProfile> rotation) {
    log(INFO, "Entering controller state: Final rotation");
    softReset({m_setpoint, rotation->getCurrentOrientation()});
    startDisplacement<StateRotation>(std::move(rotation), m_maxSpeeds.angular, m_maxAccelerations.angular);
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::reset(Position2D<Meter> robotPosition) {
    ControllerStatus status = getStatus();
    if (status != ControllerStatus::Still) {
        log(WARN, "Controller reset while the robot was moving.");
    }

    softReset(robotPosition);
    m_speedControl = false;
    m_actualSpeed.reset(robotPosition);
    m_trackingPointSpeed.reset(applyOffset(robotPosition));
    setCurrentState<StateStandStill>(robotPosition);
}

// Private
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::softReset(Position2D<Meter> robotPosition) {
    m_reversing = false;
    m_setpoint = robotPosition;
    m_goalPointSpeed.reset(applyOffset(robotPosition));
    m_converter.reset();
}

// Getters and setters
template <ErrorConverter TConverter>
ControllerStatus UnicycleController<TConverter>::getStatus() const {
    if (m_suspendedState) {
        return m_suspendedState->getStatus() | getCurrentState().getStatus();
    }
    return getCurrentState().getStatus();
}

template <ErrorConverter TConverter>
UpdateResultCode UnicycleController<TConverter>::getLastEvent() const {
    return m_event;
}

template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getTrackingOffset() const {
    return m_offset;
}

template <ErrorConverter TConverter>
bool UnicycleController<TConverter>::isSpeedControlled() const {
    return m_speedControl;
}

template <ErrorConverter TConverter>
Position2D<Meter> UnicycleController<TConverter>::getSetpoint() const {
    return m_setpoint;
}
template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getGoalPoint() const {
    return applyOffset(m_reversing ? m_setpoint.flip() : m_setpoint);
}
template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getGoalPointSpeed() const {
    return m_goalPointSpeed.value();
}

template <ErrorConverter TConverter>
Position2D<Meter, double_t> UnicycleController<TConverter>::getEstimatedRobotSpeed() const {
    return m_actualSpeed.value();
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getEstimatedRelativeRobotSpeed() const {
    return Speeds(m_actualSpeed.getLastInput().makeRelative(m_actualSpeed.value()).x, m_actualSpeed.value().theta);
}

template <ErrorConverter TConverter>
const TConverter &UnicycleController<TConverter>::getErrorConverter() const {
    return m_converter;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setErrorConverter(TConverter converter) {
    m_converter = std::move(converter);
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getBrakeAccelerations() const {
    return m_brakeAccelerations;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setBrakeAccelerations(Speeds brakeAccelerations) {
    m_brakeAccelerations = brakeAccelerations;
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getMaxSpeeds() const {
    return m_maxSpeeds;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setMaxSpeeds(Speeds speeds, bool persist) {
    if (persist) {
        m_maxSpeeds = speeds;
    }
    getCurrentState().notify(MaxSpeedsChanged(speeds));
}

template <ErrorConverter TConverter>
Accelerations UnicycleController<TConverter>::getMaxAccelerations() const {
    return m_maxAccelerations;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setMaxAccelerations(Accelerations accelerations) {
    m_maxAccelerations = accelerations;
}

// Private
template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::applyOffset(Position2D<Meter> position) const {
    return position.relativeOffset(m_offset.x, m_offset.y);
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getLastCommand() const {
    return m_lastCommand;
}

} // namespace controller

// Explicit instantiation of the controller
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/controller.hpp"
template class controller::UnicycleController<converter_t>;
