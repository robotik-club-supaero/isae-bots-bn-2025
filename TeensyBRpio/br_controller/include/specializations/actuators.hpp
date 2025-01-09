#ifndef _SPEC_ACTUATORS_HPP_
#define _SPEC_ACTUATORS_HPP_

#ifdef _SIMULATION

#include "motors/MotorStub.hpp"

using actuators_t = MotorStub;

#elif defined(ARDUINO)

#include "motors/MotorsOdrive2.hpp"

using actuators_t = MotorsOdrive2;

#else
#error "Set _SIMULATION to 1 or add an implementation of Actuators for the current platform."
#endif

#endif
