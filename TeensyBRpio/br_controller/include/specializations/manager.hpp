#ifndef _SPEC_MANAGER_HPP_
#define _SPEC_MANAGER_HPP_

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
#include "specializations/feedback.hpp"

#include "manager/ControllerManager.hpp"

using manager_t = manager::ControllerManager<actuators_t, controller_t, feedback_t>;

#if !defined(ARDUINO) && defined(_SIMULATION)
/// Convenience method to create the manager
inline manager_t createManager(Position2D<Meter> initialPosition = {}) {
    manager_t manager;
    manager.resetPosition(initialPosition);
    manager.setActive(true);
    while (!manager.isActive()) {
        manager.update(UPDATE_INTERVAL / 1e6);
    }
    return manager;
}
#endif

#endif