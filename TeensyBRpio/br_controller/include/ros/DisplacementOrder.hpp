#ifndef _ROS_POSITION_ORDER_HPP_
#define _ROS_POSITION_ORDER_HPP_

#include "Actuators.hpp"
#include "Clock.hpp"
#include "PositionFeedback.hpp"
#include "controller/UnicycleController.hpp"
#include "geometry/Position2D.hpp"
#include "manager/ControllerManager.hpp"
#include "specializations/controller.hpp"

#include <vector>

enum GoalType {
    UNVALID_GOALTYPE = -1,
    /// Linear displacement with final orientation
    LINEAR_FINAL = 0,
    /// Linear displacement without final orientation
    LINEAR_TRANS = 1,
    /// Rotation without displacement
    ORIENTATION = 9,
    /// Linear displacement backwards
    LINEAR_REVERSE = 8,

    PATH_ADD_POINT = 20,
    PATH_START_FORWARD = 21,
    PATH_START_BACKWARD = 22,
    PATH_RESET = 23,

    /// Emergency braking
    STOP = 2,
    /// Resets the estimated position
    RESET = 3,

    /// Direct control of the setpoint's speed.
    CONTROL = 4,
};

class DisplacementOrder {
  public:
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    using manager_t = manager::ControllerManager<TActuators, controller_t, TFeedback, TClock>;

    DisplacementOrder(GoalType type, Position2D<Millimeter> goalPosition);
    DisplacementOrder(int type, Position2D<Millimeter> goalPosition);

    /// Sends the order to the manager or the controller depending on the goal type.
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    void operator()(manager_t<TActuators, TFeedback, TClock> &manager, std::vector<Point2D<Meter>> &pendingPath) const;

    GoalType type;
    Position2D<Millimeter> position;

  private:
    static constexpr int CONTROL_MAX_SPEED = 255;

    /// Starts linear displacement
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    static void goTo(manager_t<TActuators, TFeedback, TClock> &manager, DisplacementKind kind, Point2D<Millimeter> goalPosition,
                     std::optional<Angle> finalOrientation);

    /// Starts path
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    static void startPath(manager_t<TActuators, TFeedback, TClock> &manager, DisplacementKind kind, std::vector<Point2D<Meter>> &path,
                          std::optional<Angle> finalOrientation);
};

#endif