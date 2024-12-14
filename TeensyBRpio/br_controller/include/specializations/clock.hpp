#ifndef _SPEC_CLOCK_HPP_
#define _SPEC_CLOCK_HPP_

#include "Clock.hpp"

#ifdef USE_TEST_CLOCK

#include "TestClock.hpp"
using _clock_t = TestClock;

#else

using _clock_t = SystemClock;

#endif

#endif
