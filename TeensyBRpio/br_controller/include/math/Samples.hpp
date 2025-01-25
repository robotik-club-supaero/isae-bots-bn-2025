#ifndef _SAMPLES_HPP_
#define _SAMPLES_HPP_

#include "defines/math.hpp"
#include <vector>

/**
 * Contains sampled values over an interval, with constant step size.
 *
 * If `[a, b]` is the sampling interval and `n` is the number of samples, then the `i-th` value is
 * the value at point `a + (b - a) * i / (n-1)`.
 */
template <typename T = double_t>
class Samples {
  public:
    using size_type = std::size_t;

    /**
     * Creates empty samples (i.e. `numOfSamples() == 0`).
     * `domainStart()` and `domainEnd()` are initialized to 0.
     */
    Samples() = default;

    /**
     * @param function The function to sample
     * @param domainStart,domainEnd The endpoints of the function's domain of definition.
     * @param domainLen The number of sampling steps. `function` will be called exactly `domainLen + 1` times.
     *
     * The behavior is undefined if `domainStart > domainEnd` or (`domainStart == domainEnd` and `numOfSteps > 0`).
     */
    template <typename Fun>
        requires std::is_invocable_r_v<T, Fun, double_t>
    Samples(const Fun &function, double_t domainStart, double_t domainEnd, size_type numOfSteps)
        : m_domainStart(domainStart), m_domainEnd(domainEnd), m_samples() {
        if (numOfSteps == 0) {
            m_samples.push_back(function(domainStart));
            return;
        }
        m_samples.reserve(numOfSteps + 1);
        double_t step = (domainEnd - domainStart) / numOfSteps;
        for (size_type i = 0; i <= numOfSteps; i++) {
            m_samples.push_back(function(domainStart + i * step));
        }
    }
    /**
     * @param samples Vector of pre-sampled values between `domainStart` and `domainEnd`.
     *
     * The behavior is undefined if `samples` is empty, `domainStart > domainEnd` or (`domainStart == domainEnd` and `samples.size() != 1`).
     */
    Samples(std::vector<T> samples, double_t domainStart, double_t domainEnd)
        : m_domainStart(domainStart), m_domainEnd(domainEnd), m_samples(std::move(samples)) {}

    /**
     * Returns a reference to the last sampled value before domain point `arg`.
     *
     * The behavior is undefined if `numOfSamples() == 0`, `arg < domainStart()` or `arg > domainEnd()`.
     */
    /// @{
    const T &operator()(double_t arg) const { return this->operator[](indexOf(arg)); }
    T &operator()(double_t arg) { return this->operator[](indexOf(arg)); }
    ///@}

    /**
     * Returns a reference to the sampled value at position `index`.
     *
     * The behavior is undefined if `index >= numOfSamples()`.
     */
    /// @{
    const T &operator[](size_type index) const { return m_samples[index]; }
    T &operator[](size_type index) { return m_samples[index]; }
    /// @}

    /**
     * Interpolates the value at point `arg` based on the immediately preceding and immediately following sampled values.
     *
     * The behavior is undefined if `numOfSamples() == 0`, `arg < domainStart()` or `arg > domainEnd()`.
     */
    T interpolate(double_t arg) const {
        size_type index = indexOf(arg);
        double_t argLow = domainPointAt(index);
        if (argLow < arg && index + 1 < numOfSamples()) {
            double_t argHigh = domainPointAt(index + 1);
            T low = this->operator[](index);
            T high = this->operator[](index + 1);
            return low + (high - low) * (arg - argLow) / (argHigh - argLow);
        } else {
            return this->operator[](index);
        }
    }

    /**
     * Returns the index of the greatest sampling point that is less than `arg`.
     *
     * The result is unspecified if `numOfSamples() == 0`, `arg < domainStart()` or `arg > domainEnd()`.
     */
    size_type indexOf(double_t arg) const { return std::floor(doubleIndexOf(arg)); }

    /**
     * Returns the index of the smallest sampling point that is greater than `arg`.
     *
     * The result is unspecified if `numOfSamples() == 0`, `arg < domainStart()` or `arg > domainEnd()`.
     */
    size_type ceilingIndexOf(double_t arg) const { return std::ceil(doubleIndexOf(arg)); }

    /**
     * Returns the domain point that corresponds to the `index`-th sampled value.
     *
     * The result is unspecified if `index >= numOfSamples()`.
     */
    double_t domainPointAt(size_type index) const {
        size_type numSamples = numOfSamples();
        if (numSamples == 1) {
            return m_domainStart;
        }
        return m_domainStart + index * (m_domainEnd - m_domainStart) / (numSamples - 1);
    }

    double_t domainStart() const { return m_domainStart; }
    double_t domainEnd() const { return m_domainEnd; }
    size_type numOfSamples() const { return m_samples.size(); }

  private:
    inline double_t doubleIndexOf(double_t arg) const {
        size_type numSamples = numOfSamples();
        if (numSamples == 1) {
            return 0;
        }
        return (arg - m_domainStart) / (m_domainEnd - m_domainStart) * (numSamples - 1);
    }

    double_t m_domainStart;
    double_t m_domainEnd;
    std::vector<T> m_samples;
};

#endif