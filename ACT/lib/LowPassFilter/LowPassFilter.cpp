#include "LowPassFilter.hpp"

#include <cmath>

LowPassFilter::LowPassFilter(double tau) : m_tau(tau), m_output() {}

void LowPassFilter::update(double value, double interval) {
    if (!std::isnan(value) && !std::isinf(value)) {
        m_output = (value + (m_output * m_tau) / interval) / (1.0 + m_tau / interval);  // we consider dS/dt = (U_(n+1) - U_n) / dt
    }
}

LowPassFilter::operator double() const { return m_output; }

double LowPassFilter::value() const { return m_output; }

void LowPassFilter::reset() { m_output = 0.; }