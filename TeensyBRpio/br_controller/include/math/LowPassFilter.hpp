#ifndef _LOW_PASS_FILTER_HPP_
#define _LOW_PASS_FILTER_HPP_

#include "defines/math.hpp"

/**
 * First-order low pass fiter.
 */
template <typename TValue = number_t>
    requires Add<TValue> && Mul<TValue> && std::is_default_constructible_v<TValue>
class LowPassFilter {
  public:
    LowPassFilter(number_t tau);

    /**
     * Updates the output of the filter
     * @param value The current value of the function.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(TValue input, number_t interval);

    TValue value() const;
    operator TValue() const;

  private:
    number_t m_tau;  // time constant, in s
    TValue m_output; // last output that has been computed
};

#endif
