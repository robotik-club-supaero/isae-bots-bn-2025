#ifndef _TEST_CLOCK_HPP_
#define _TEST_CLOCK_HPP_

#include "Clock.hpp"
#include <iostream>

class TestClock {
  public:
    TestClock() = default;

    instant_t micros() const {
        if (m_paused) {
            return m_pausedTime;
        }
        return m_clock.micros() - m_offset;
    }

    void pause() {
        if (m_paused) {
            return;
        }
        m_pausedTime = m_clock.micros();
        m_paused = true;
    }

    void resume() {
        if (!m_paused) {
            return;
        }
        m_paused = false;
        m_offset += getDurationMicros(m_pausedTime, m_clock.micros());
    }

    bool paused() const { return m_paused; }

  private:
    SystemClock m_clock;
    bool m_paused;
    instant_t m_pausedTime;
    duration_t m_offset;
};

#endif