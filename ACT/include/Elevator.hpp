#ifndef _ELEVATOR_HPP_
#define _ELEVATOR_HPP_

#include <Stepper.h>

#include "ActuatorState.hpp"

enum ElevatorOrder : uint16_t { MOVE_DOWN = 0, MOVE_UP = 1 };

enum ElevatorCallback : uint16_t { DOWN = 0, UP = 1 };

class ElevatorStepper {
   public:
    ElevatorStepper(int number_of_steps, int pin1, int pin2, int level, long speed, int move_steps);

    ElevatorCallback getState() const;
    void setState(uint16_t state);

   private:
    Stepper m_stepper;
    int m_level;
    int m_steps;
    ElevatorCallback m_state;
};

class ElevatorStepper1 : public ElevatorStepper {
   public:
    ElevatorStepper1();
};

class ElevatorStepper2 : public ElevatorStepper {
   public:
    ElevatorStepper2();
};

class Elevators {
   public:
    Elevators(ros2::Node &node);

    void loop();

   private:
    ElevatorStepper1 m_stepper_1;
    ElevatorStepper2 m_stepper_2;

    ActuatorStateManager m_elevator_1;
    ActuatorStateManager m_elevator_2;
};

#endif