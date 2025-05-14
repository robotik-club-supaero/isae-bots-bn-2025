
#include "ODriveArduino.h"

// Print with stream operator
template<typename T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, double_t arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}

void ODriveArduino::SetPosition(int motor_number, double_t position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, double_t position, double_t velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, double_t position, double_t velocity_feedforward, double_t current_feedforward) {
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, double_t velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, double_t velocity, double_t current_feedforward) {
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, double_t current) {
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, double_t position) {
    serial_ << "t " << motor_number << " " << position << "\n";
}

double_t ODriveArduino::readFloat() {
    return readString().toFloat();
}

double_t ODriveArduino::GetVelocity(int motor_number) {
	serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODriveArduino::readFloat();
}

double_t ODriveArduino::GetPosition(int motor_number) {
    serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODriveArduino::readFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}


String ODriveArduino::test() {
    serial_<< "r vbus_voltage\n";
    return readString();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, double_t timeout) {
    int timeout_ctr = (int)(timeout * 100.0f);
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(10);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != requested_state && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

int ODriveArduino::getCurrentAxisState(int axis) {
    
    serial_ << "r axis" << axis << ".current_state\n";
    return readInt();
}


String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 2000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
