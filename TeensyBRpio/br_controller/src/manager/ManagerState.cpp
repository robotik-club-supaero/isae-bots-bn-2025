#include "manager/ManagerState.hpp"
#include "geometry/Position2D.hpp"

namespace manager {

template <Actuators TActuators>
ManagerState<TActuators>::ManagerState() {}

} // namespace manager

#include "specializations/actuators.hpp"
template class manager::ManagerState<actuators_t>;