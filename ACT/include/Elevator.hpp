#ifndef _ELEVATOR_HPP_
#define _ELEVATOR_HPP_

#include "Actuator.hpp"

enum ElevatorOrder : uint16_t { MOVE_DOWN = 0, MOVE_UP = 1 };

enum ElevatorCallback : uint16_t { DOWN = 0, UP = 1 };

class ElevatorHandle : public ActuatorHandle<2> {
   public:
    ElevatorHandle(int servo_pin, const char *name, std::array<int, 2> positions);

    bool hasError() const;
    void setHasError(bool error);

   private:
    bool m_was_refused;
};

class Elevators {
   public:
    Elevators(ros2::Node &node);

    void loop();

   private:
    static bool checkStates(const ElevatorHandle &handle1, const ElevatorHandle &handle2, int level, uint16_t new_state);
    static void loop(ElevatorHandle &handle, ActuatorTopics &topics);

    static std::function<void(uint16_t)> createCallback(const std::shared_ptr<ElevatorHandle> &handle1,
                                                        const std::shared_ptr<ElevatorHandle> &handle2, int level);

    std::shared_ptr<ElevatorHandle> m_handle_1;
    std::shared_ptr<ElevatorHandle> m_handle_2;

    ActuatorTopics m_elevator_1;
    ActuatorTopics m_elevator_2;
};

#endif