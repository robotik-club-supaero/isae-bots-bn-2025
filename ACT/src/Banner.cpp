#include "Banner.hpp"

#include "configuration.hpp"

BannerHandle::BannerHandle(int servo_pin, const char* name, std::array<int, 2> positions)
    : ActuatorHandle<2>(servo_pin, name, positions, /* initial_state = */ RETRACTED) {}

Banner::Banner(ros2::Node& node)
    : m_handle_1(std::make_shared<BannerHandle>(BANNER_1_PIN, "BANNER1", std::array<int, 2>{BANNER_1_RETRACTED_POS, BANNER_1_DEPLOYED_POS})),
      m_handle_2(BANNER_2_PIN, "BANNER2", std::array<int, 2>{BANNER_2_RETRACTED_POS, BANNER_2_DEPLOYED_POS}),
      m_ros(node, "/act/order/banderolle", "/act/callback/banderolle", [handle = std::weak_ptr<BannerHandle>(m_handle_1)](uint16_t order) {
          if (auto handle1 = handle.lock()) {
              if (order == DEPLOY && handle1->getState() == RETRACTED) {
                  handle1->setState(DEPLOYED);
              }
          }
      }) {}

void Banner::loop() {
    if (m_handle_1->needsNotify() && micros() - m_handle_1->lastChangeTime() > BANNER_INTERVAL) {
        m_handle_2.setState(DEPLOYED);
        m_handle_1->markNotified();
    } else if (m_handle_2.needsNotify() && micros() - m_handle_2.lastChangeTime() > CALLBACK_INTERVAL) {
        m_ros.sendCallback(DEPLOYED);
        m_handle_2.markNotified();
    }
}