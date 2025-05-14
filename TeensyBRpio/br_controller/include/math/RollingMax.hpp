#ifndef _DYNAMIC_MAX_HPP_
#define _DYNAMIC_MAX_HPP_

#include "defines/math.hpp"
#include "stl/SmallDeque.hpp"

/**
 * Estimation of the maximum of a function on compact variable-length intervals of which the start edge is non-decreasing.
 *
 * This class divides the domain of definition of the functions in subdomains where the function is monotonic. Space and amortized time complexity
 * depend only on the number of monotonic segments, not the length of the domain. Worst-case time complexity is O(length/step) which occurs when
 * sampling is required - either because it hasn't been performed yet, or because previously sampled values could not be stored due to insufficient
 * memory
 *
 * For example, if `f(x) = x²` on [-1, 1], then f is decreasing on [-1, 0] and increasing on [0, 1]. As a result,
 * the maximum of `f` on [a, b] is
 * - f(a) if -1 <= a < b <= 0
 * - f(b) if 0 <= a < b <= 1
 * - max(f(a), f(b)) otherwise.
 *
 */
class RollingMax {
  public:
    RollingMax(double_t step, double_t initialStart = 0)
        : m_start(initialStart), m_end(initialStart), m_step(step), m_extrema(), m_start_increasing() {}

    /**
     * Estimates the maximum of `function` on `[start, end]`.
     *
     * The function is guaranteed not to be called with a value less than `start`. However, it may be called with a value in `[end, end + step]` due
     * to sampling.
     * The behavior is undefined if `end` < `start` or `start` is less than `initialStart` or the value used in the previous call to `getMaximum`.
     */
    template <typename Fun>
        requires std::is_invocable_r_v<double_t, Fun &, double_t>
    double_t getMaximum(double_t start, double_t end, Fun &&function) {
        shrinkInterval(start);
        bool sample_complete = sampleUntil(end, function);

        bool increasing = m_start_increasing;

        double_t value = increasing ? std::numeric_limits<double_t>::lowest() : function(start);
        for (std::size_t i = 0; i < m_extrema.size() && m_extrema[i] < end; i++) {
            if (increasing && m_extrema[i] >= start) {
                value = std::max(value, function(m_extrema[i]));
            }
            increasing = !increasing;
        }
        if (increasing) {
            value = std::max(value, function(sample_complete ? end : m_end));
        }

        // If memory was insufficient to extend the interval, fallback to naive linear search
        if (!sample_complete) {
            value = std::max(value, getMaximumNaive(end, std::forward<Fun>(function)));
        }
        return value;
    }

    const SmallDeque<double_t> &getExtrema() const { return m_extrema; }
    bool isStartIncreasing() const { return m_start_increasing; }

  private:
    template <typename Fun>
        requires std::is_invocable_r_v<double_t, Fun &, double_t>
    double_t getMaximumNaive(double_t end, Fun &&function) const {
        double_t m = std::numeric_limits<double_t>::lowest();
        for (double_t x = m_end + m_step; x <= end; x += m_step) {
            m = std::max(m, function(x));
        }
        return m;
    }

    void shrinkInterval(double_t newStart) {
        while (!m_extrema.empty() && m_extrema.front() < newStart) {
            // The start edge of the interval is required to be non-decreasing, so we drop the obsolete extremums
            m_extrema.pop_front();
            m_start_increasing = !m_start_increasing;
        }
        m_start = newStart;
    }

    template <typename Fun>
        requires std::is_invocable_r_v<double_t, Fun &, double_t>
    bool sampleUntil(double_t newEnd, Fun &&function) {
        if (newEnd <= m_end) {
            return true;
        }
        if (m_end == m_start) {
            m_end += m_step;
            double_t initialValue = function(m_start);
            double_t endValue = function(m_end);
            m_start_increasing = endValue >= initialValue;
        }

        bool m_end_increasing = m_start_increasing ^ (m_extrema.size() % 2 == 1);

        double_t oldEndValue = function(m_end);
        while (newEnd > m_end) {
            double_t nextEnd = m_end + m_step;
            double_t endValue = function(nextEnd);

            if (m_end_increasing) {
                if (oldEndValue > endValue) {
                    if (!addExtremum(m_end)) {
                        return false;
                    }
                    m_end_increasing = false;
                }
            } else if (oldEndValue <= endValue) {
                if (!addExtremum(nextEnd)) {
                    return false;
                }
                m_end_increasing = true;
            }

            m_end = nextEnd;
            oldEndValue = endValue;
        }

        return true;
    }

    bool addExtremum(double_t x) { return m_extrema.push_back(x); }

    double_t m_start;
    double_t m_end;
    double_t m_step;
    SmallDeque<double_t> m_extrema;

    /// Whether the function is initially increasing (or constant) around `m_start` (false = decreasing)
    bool m_start_increasing;
};

#endif