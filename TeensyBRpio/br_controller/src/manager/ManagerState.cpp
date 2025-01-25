#include "manager/ManagerState.hpp"
#include "geometry/Position2D.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
ManagerState<TActuators, TController>::ManagerState() {}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::ManagerState<actuators_t, controller_t>;