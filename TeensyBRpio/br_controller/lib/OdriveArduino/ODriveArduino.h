
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"
#include "ODriveEnums.h"

class ODriveArduino {
public:
    ODriveArduino(Stream& serial);

    // Commands
    void SetPosition(int motor_number, double_t position);
    void SetPosition(int motor_number, double_t position, double_t velocity_feedforward);
    void SetPosition(int motor_number, double_t position, double_t velocity_feedforward, double_t current_feedforward);
    void SetVelocity(int motor_number, double_t velocity);
    void SetVelocity(int motor_number, double_t velocity, double_t current_feedforward);
    void SetCurrent(int motor_number, double_t current);
    void TrapezoidalMove(int motor_number, double_t position);
    // Getters
    double_t GetVelocity(int motor_number);
    double_t GetPosition(int motor_number);
    // General params
    double_t readFloat();
    int32_t readInt();

    String test();

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, double_t timeout = 10.0f);

    int getCurrentAxisState(int axis);


private:
    String readString();

    Stream& serial_;
};

#endif //ODriveArduino_h
