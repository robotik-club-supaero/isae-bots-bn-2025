#ifndef _DYNAMIC_MAX_HPP_
#define _DYNAMIC_MAX_HPP_

#include "defines/math.hpp"
#include "math/Samples.hpp"

/**
 * Amortized constant-time estimation of the maximum of a function on compact variable-length intervals.
 *
 * This samples the function and divides its domain of definition in subdomains where the function is monotonic.
 *
 * For example, if `f(x) = x²` on [-1, 1], then f is decreasing on [-1, 0] and increasing on [0, 1]. As a result,
 * the maximum of `f` on [a, b] is
 * - f(a) if -1 <= a < b <= 0
 * - f(b) if 0 <= a < b <= 1
 * - max(f(a), f(b)) otherwise.
 */
class RollingMax {
  public:
    using size_type = std::size_t;

    RollingMax() = default;

    /**
     * @param samples Pre-samples values of the function
     */
    RollingMax(const Samples<double_t> &samples);

    /**
     * Returns an estimation of the maximum of the function on interval [start, end].
     *
     * If `start < domainStart`, `end > domainEnd`, `start > end` or `numOfSamples() == 0`, the behavior is undefined.
     */
    double_t getMaximum(double_t start, double_t end) const;

    /**
     * Returns an estimation of the function's value at point `arg`.
     *
     * The behavior is undefined if `arg < domainStart()`, `arg > domainEnd()` or `numOfSamples() == 0`.
     */
    double_t operator()(double_t arg) const;

    double_t domainStart() const;
    double_t domainEnd() const;
    size_type numOfSamples() const;

  private:
    struct Evaluation {
        double_t value;
        size_type subdomainIndex;
    };
    enum MonotonicityKind { INCREASING, DECREASING, CONSTANT };
    struct Subdomain {
        MonotonicityKind kind;
        size_type end;
    };

    Samples<Evaluation> m_samples;
    std::vector<Subdomain> m_subdomains;
};

#endif