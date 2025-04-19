#include "Banner.hpp"

#include "configuration.hpp"
#include "logging.hpp"

BannerServo::BannerServo(int servo_pin, std::array<int, 2> positions) : ActuatorServo<2>(servo_pin, positions, /* initial_state = */ RETRACTED) {}

BannerServo1::BannerServo1() : BannerServo(BANNER_1_PIN, std::array<int, 2>{BANNER_1_RETRACTED_POS, BANNER_1_DEPLOYED_POS}) {}
BannerServo2::BannerServo2() : BannerServo(BANNER_2_PIN, std::array<int, 2>{BANNER_2_RETRACTED_POS, BANNER_2_DEPLOYED_POS}) {}

Banner::Banner(ros2::Node& node) : m_servo_1(), m_servo_2(), m_ros(node, "/act/order/banderolle", "/act/callback/banderolle") {}

void Banner::loop() {
    if (auto state = m_ros.getRequestedState()) {
        if (*state == DEPLOY) {
            log(INFO, "Deploying banner");
            m_servo_1.setState(DEPLOYED);
        } else {
            log(WARN, "Invalid order received for banner");
        }
        m_ros.clearRequestedState();
    } else if (m_servo_1.needsNotify() && micros() - m_servo_1.getLastChangeTime() > BANNER_INTERVAL) {
        log(DEBUG, "Deploying banner (phase 2)");
        m_servo_2.setState(DEPLOYED);
        m_servo_1.markNotified();
    } else if (m_servo_2.needsNotify() && micros() - m_servo_2.getLastChangeTime() > CALLBACK_INTERVAL) {
        m_ros.sendCallback(DEPLOYED);
        m_servo_2.markNotified();
    }
}