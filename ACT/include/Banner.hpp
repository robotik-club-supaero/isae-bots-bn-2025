#ifndef _BANNER_HPP_
#define _BANNER_HPP_

#include "Actuator.hpp"

enum BannerOrder : uint16_t { DEPLOY = 1 };

enum BannerCallback : uint16_t { RETRACTED = 0, DEPLOYED = 1 };

class BannerHandle : public ActuatorHandle<2> {
   public:
    BannerHandle(int servo_pin, const char* name, std::array<int, 2> positions);
};

class Banner {
   public:
    Banner(ros2::Node& node);

    void loop();

   private:
    std::shared_ptr<BannerHandle> m_handle_1;
    BannerHandle m_handle_2;

    ActuatorTopics m_ros;
};

#endif