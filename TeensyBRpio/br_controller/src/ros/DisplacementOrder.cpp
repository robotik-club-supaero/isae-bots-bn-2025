#include "ros/DisplacementOrder.hpp"
#include "logging.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/LinearTrajectory.hpp"
#include "trajectories/PathTrajectory.hpp"

DisplacementOrder::DisplacementOrder(GoalType type, Position2D<Millimeter> goalPosition) : type(type), position(goalPosition) {}

DisplacementOrder::DisplacementOrder(int type, Position2D<Millimeter> goalPosition) : DisplacementOrder(UNVALID_GOALTYPE, goalPosition) {
    switch (type) {
        case LINEAR_FINAL:
        case LINEAR_TRANS:
        case ORIENTATION:
        case LINEAR_REVERSE:
        case STOP:
        case RESET:
        case CONTROL:
        case PATH_ADD_POINT:
        case PATH_START_FORWARD:
        case PATH_START_BACKWARD:
        case PATH_RESET:
            this->type = (GoalType)type;
            break;
        default:
            log(WARN, "Order ignored because not recognized: " + std::to_string(type));
            break;
    }
}

template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
void DisplacementOrder::operator()(manager_t<TActuators, TFeedback, TClock> &manager, std::vector<Point2D<Meter>> &pendingPath) const {
    switch (type) {
        case LINEAR_FINAL:
            goTo(manager, FORWARD, position, position.theta);
            break;
        case LINEAR_TRANS:
            goTo(manager, FORWARD, position, {});
            break;
        case LINEAR_REVERSE:
            goTo(manager, REVERSE, position, {});
            break;
        case ORIENTATION:
            manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) {
                controller.template startRotation<SetHeadingProfile>(robotPosition.theta, position.theta);
            });
            break;
        case STOP:
            manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); });
            break;
        case RESET:
            manager.resetPosition(position.toMeters());
            break;
        case CONTROL: {
            Vector2D<Millimeter> speedFactor = static_cast<Vector2D<Millimeter>>(position) / CONTROL_MAX_SPEED;
            manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) {
                controller.setSetpointSpeed({speedFactor.x * controller.getMaxSpeeds().linear,
                                             speedFactor.y * controller.getMaxSpeeds().angular} /*, enforceMaxSpeeds = true */);
            });
            break;
        }
        case PATH_ADD_POINT:
            addPathPoint(pendingPath, position.toMeters());
            if (position.theta.value() == 1) {
                startPath(manager, FORWARD, pendingPath, {});
            } else if (position.theta.value() == 2) {
                startPath(manager, REVERSE, pendingPath, {});
            } else if (position.theta.value() != 0) {
                log(WARN, "Invalid theta value in PATH_ADD_POINT order; this could lead to unexpected behavior in the future");
            }
            break;
        case PATH_START_FORWARD:
        case PATH_START_BACKWARD: {
            DisplacementKind kind = (type == PATH_START_BACKWARD) ? REVERSE : FORWARD;
            addPathPoint(pendingPath, position.toMeters());
            startPath(manager, kind, pendingPath, position.theta);
            break;
        }
        case PATH_RESET:
            pendingPath.clear();
            break;
        default:
            break;
    }
}

template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
void DisplacementOrder::goTo(manager_t<TActuators, TFeedback, TClock> &manager, DisplacementKind kind, Point2D<Millimeter> goalPosition,
                             std::optional<Angle> finalOrientation) {
    manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) {
        controller.startTrajectory(kind, std::make_unique<LinearTrajectory>(robotPosition, goalPosition.toMeters()), finalOrientation);
    });
}

template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
void DisplacementOrder::startPath(manager_t<TActuators, TFeedback, TClock> &manager, DisplacementKind kind, std::vector<Point2D<Meter>> &path,
                                  std::optional<Angle> finalOrientation) {
    manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) {
        path.insert(path.begin(), robotPosition);
        Angle initialDirection = robotPosition.theta + kind.getAlignmentOffset();
        controller.startTrajectory(kind, std::make_unique<PathTrajectory>(initialDirection, std::move(path)), finalOrientation);
    });
}

void DisplacementOrder::addPathPoint(std::vector<Point2D<Meter>> &path, Point2D<Meter> point) {
    if (path.empty() || path.back() != point) {
        path.push_back(point);
    } else {
        log(WARN, "Point " + std::string(point) + " ignored because it is already added to the path.");
    }
}

#include "specializations/manager.hpp"
template void DisplacementOrder::operator()<actuators_t, feedback_t, _clock_t>(::manager_t &manager, std::vector<Point2D<Meter>> &pendingPath) const;