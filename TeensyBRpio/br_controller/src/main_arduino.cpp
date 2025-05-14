#include "specializations/ros.hpp"
#include <Led.hpp>

std::optional<BlinkLED> blinkingLed = std::nullopt;
std::optional<ros_t> rosInstance = std::nullopt;
uint32_t reportTime = 0;

void setup() {
    Serial.begin(115200);
    ros2::init(Serial);

    blinkingLed.emplace();

    rosInstance.emplace();
    rosInstance->attachManager();
}

void loop() {
    rosInstance->loop();
    blinkingLed->loop();

    uint32_t time = millis();
    if ((time - reportTime) > 2000) {
        reportTime = time;
        log(INFO, "USED POOL: " + to_string(ALLOCATOR.memory_usage_percentage()));
    }
}

void log(LogSeverity severity, const char *message) {
    if (rosInstance) {
        rosInstance->sendLog(severity, message);
    }
}