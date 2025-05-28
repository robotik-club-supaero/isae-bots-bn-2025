#ifndef _ELEVATOR_HPP_
#define _ELEVATOR_HPP_

#include <Stepper.h>

#include <array>

#include "ActuatorState.hpp"

enum ElevatorOrder : uint16_t { MOVE_DOWN = 0, MOVE_MIDDLE = 1, MOVE_UP = 2 };

enum ElevatorCallback : uint16_t { DOWN = 0, MIDDLE = 1, UP = 2 };

/// Converts between statess (UP/DOWN) and the stepper command. Don't forget to call `loop`.
class ElevatorStepper {
   public:
    /// @param number_of_steps Number of steps per revolution (resolution)
    /// @param pin1 Pin 1 (STEP) of the stepper
    /// @param pin2 Pin 2 (DIR) of the stepper
    /// @param level Level of the elevator (1 or 2)
    /// @param speed Number of revolutions per second when moving
    /// @param move_steps Number of steps to do when moving from DOWN to UP
    ElevatorStepper(int number_of_steps, int pin1, int pin2, int level, long speed, std::array<int, 2> positions);

    ElevatorCallback getState() const;

    /// Sets the desired state of the stepper.
    /// Warning: the stepper does not actually move until `loop` is called.
    bool setState(uint16_t state);

    /// Resets the simulated position without moving the stepper.
    /// Also stops the stepper if it was moving.
    void reset(ElevatorCallback state);

    /// Effectively moves the stepper if appropriate. If no move was requested, this is a no-op.
    /// This will block for approximately `STEPPER_YIELD_TIMEOUT` in the worst case (see configuration.hpp).
    /// @returns true when the stepper is done moving, so a ROS callback can be sent.
    bool loop();

   private:
    int getCurrentPosition() const;
    int getPositionOfState(ElevatorCallback state) const;

    Stepper m_stepper;
    int m_level;
    std::array<int, 2> m_positions;
    int m_remaining_steps;
    int m_max_steps;
    ElevatorCallback m_state;
};

/// An `ElevatorStepper` that uses the settings of ELEVATOR_1 (lower level). See `ElevatorStepper` and `configuration.hpp`.
class ElevatorStepper1 : public ElevatorStepper {
   public:
    ElevatorStepper1();
};

/// An `ElevatorStepper` that uses the settings of ELEVATOR_2 (upper level). See `ElevatorStepper` and `configuration.hpp`.
class ElevatorStepper2 : public ElevatorStepper {
   public:
    ElevatorStepper2();
};

/// Connects the ROS to the steppers. Don't forget to call `loop`.
class Elevators {
   public:
    Elevators(ros2::Node &node);

    void loop();
    void reset();

   private:
    ElevatorStepper1 m_stepper_1;
    ElevatorStepper2 m_stepper_2;

    ActuatorStateManager m_elevator_1;
    ActuatorStateManager m_elevator_2;
};

#endif