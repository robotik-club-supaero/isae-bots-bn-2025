#ifndef _CURVE_SAMPLING_HPP_
#define _CURVE_SAMPLING_HPP_

#include "geometry/Vector2D.hpp"
#include "stl/SmallDeque.hpp"

#include <concepts>

/// A differentiable parametric curve
template <typename T>
concept Curve = requires(const T &c, double_t t) {
    { c(t) } -> std::convertible_to<Point2D<Meter>>;
    { c.derivative(t) } -> std::convertible_to<Vector2D<Meter>>;
    { c.curvature(t) } -> std::convertible_to<double_t>;
};

/// Adaptive yet robust curve sampling suitable for low memory environments.
///
/// This class implements:
/// - length estimation
/// - amortized constant-time inverse arc length estimation (parameter t as a function of the distance).
///
/// Memory usage only depends on the complexity of the curve, not its length. Perfect lines don't require any dynamic allocation at all.
/// Worst-case time complexity is O(length/step) which occurs when sampling is required - either because it hasn't been performed yet, or because
/// previously sampled values could not be stored due to insufficient memory
class CurveSampling {
  public:
    CurveSampling(double_t step) : CurveSampling(step, step / 2) {}
    CurveSampling(double_t step, double_t tolerance)
        : m_step(step), m_tolerance(tolerance), m_pos(), m_midt(), m_detachedlength(UNDEFINED), m_samples() {}

    /// Samples the curve over the specified distance (starting from the beginning of the curve). If the specified distance was already sampled, this
    /// is a no-op.
    ///
    /// The behavior is undefined if `dist < 0` or `curve` is different from the curves used in previous calls to `sample` or `solveInverseArcLength`.
    ///
    /// Sampling might be aborted partway if the memory is insufficient. In this case, `isDegraded()` will be true and `sampledLength()` will be less
    /// than `dist` when this function returns.
    template <Curve T>
    void sample(const T &curve, double_t dist) {
        double_t last_sample_t = m_samples.empty() ? 0 : m_samples.back().t;

        while (!samplingComplete() && m_pos.length < dist) {
            if (m_pos.t > 0 && !std::signbit(m_midt)) {
                double_t midt_est = (last_sample_t + m_pos.t) / 2;
                if (std::abs(m_midt - midt_est) > m_tolerance) {
                    if (m_samples.push_back(m_pos)) {
                        last_sample_t = m_pos.t;
                        m_midt = m_pos.t;
                    } else {
                        markDegraded();
                        return;
                    }
                }
            }

            if (!m_pos.advance(curve, m_step)) {
                log(ERROR, "Aborting sampling of zero-length or ill-formed curve");
                break;
            }

            // Since m_midt is non-negative, we use its sign bit to track whether this is an even or odd step, so we don't waste space with an
            // additional boolean field.
            if (std::signbit(m_midt)) {
                m_midt = std::copysign(m_midt, 1);
                m_midt += m_step / curve.derivative(m_midt).norm();
            } else {
                m_midt = std::copysign(m_midt, -1);
            }
        }
        clearDegraded();
    }

    /// Whether the sampling is "degraded".
    ///
    /// This means that during a previous call to `sample` or `solveInverseArcLength`, the memory was insufficient to sample `t` properly and was
    /// aborted partway.
    ///
    /// Until this situation is solved, `solveInverseArcLength` might have terrible precision when `dist > sampledLength()`. The length of the curve
    /// may have been estimated properly if `lengthEstimationComplete()` is true. When the memory is sufficient again, this can be solved by calling
    /// `sample` or the non-const version of `solveInverseArcLength`.
    bool isDegraded() const { return m_detachedlength == DEGRADED || (!samplingComplete() && lengthEstimationComplete()); }

    /// Allows to discard any sample that is not useful to solve the inverse arc length for `dist >= before`.
    ///
    /// If, afterwards, `solveInverseArcLength` is called with a distance less than `before`, the behavior is undefined.
    void discardSamples(double_t before) {
        if (!m_samples.empty() && m_samples.front().length < before) {
            // If sample0 < sample1 < before < sample2, we can drop sample0 but we must keep sample1 because it affects estimation of `dist` in
            // [before, sample2].
            while (m_samples.size() >= 2 && m_samples[1].length < before) {
                m_samples.pop_front();
            }
        }
    }

    bool samplingComplete() const { return m_pos.t == 1; }

    bool lengthEstimationComplete() const { return samplingComplete() || m_detachedlength >= 0; }

    /// The estimated length of the curve. If `lengthEstimationComplete()` is `false`, the result is unspecified.
    double_t length() const {
        if (m_detachedlength < 0) {
            return m_pos.length;
        } else {
            return m_detachedlength;
        }
    }

    double_t sampledLength() const { return m_pos.length; }

    /// Estimates the parameter `t` such that `Integral_[0, t] ||C'(l)|| dl = dist`, where `C` is the curve that was sampled.
    ///
    /// The behavior is undefined if `dist < 0` or `dist > sampledLength() && !lengthEstimationComplete()`. If `lengthEstimationComplete()` is true
    /// but `dist > sampledLength()`, the behavior is defined but the result might be very approximate. See documentation of
    /// `solveInverseArcLength(const T&, dist)` below.
    double_t solveInverseArcLength(double_t dist) const {
        if (!samplingComplete() && dist > m_pos.length) {
            // Very rough estimation if the memory is insufficient for sampling.
            // We keep the constant-time to the detriment of the precision. We can't afford to restart the search from `m_pos` everytime this is
            // called.
            Position lastSample = m_samples.empty() ? Position() : m_samples.back();
            return interpolate(dist, lastSample, Position(m_detachedlength, 1));
        }

        std::size_t i = 0;
        while (i < m_samples.size() && m_samples[i].length < dist) {
            i++;
        }
        Position start = i == 0 ? Position() : m_samples[i - 1];
        Position end = i == m_samples.size() ? m_pos : m_samples[i];

        return interpolate(dist, start, end);
    }

    /// Estimates the parameter `t` such that `Integral_[0, t] ||C'(l)|| dl = dist` where `C` is the curve being sampled.
    ///
    /// This samples the curve first. The behavior is undefined if `dist` < 0 or `curve` is different from the curves used in previous calls to
    /// `sample` or `solveInverseArcLength`.
    ///
    /// If `sampledLength() < dist` when this returns, this means the memory was insufficient to perform an accurate sampling of the curve and the
    /// result might be very approximate. See `isDegraded` for how to solve the issue.
    template <Curve T>
    double_t solveInverseArcLength(const T &curve, double_t dist) {
        sample(curve, dist);
        if (!samplingComplete() && dist > m_pos.length) {
            computeLength(curve);
        }
        return solveInverseArcLength(dist);
    }

    std::size_t numberOfSamplePoints() const { return m_samples.size(); }

  private:
    static constexpr double_t UNDEFINED = -1;
    static constexpr double_t DEGRADED = -2;

    struct Position {
        constexpr Position() = default;
        constexpr Position(double_t length, double_t t) : length(length), t(t) {}
        double_t length;
        double_t t;

        template <Curve TCurve>
        bool advance(const TCurve &curve, double_t dist) {
            double_t derivative = curve.derivative(t).norm();
            if (derivative == 0) {
                length += Point2D<Meter>::distance(curve(t), curve(1));
                t = 1;
                return false;
            }

            length += dist;
            t += dist / derivative;

            if (t > 1) {
                length += (1 - t) * derivative;
                t = 1;
            }

            return true;
        }
    };

    static double_t interpolate(double_t dist, const Position &start, const Position &end) {
        if (end.length == start.length || dist <= start.length) {
            return start.t;
        }
        if (end.length <= dist) {
            return end.t;
        }

        double_t s = (dist - start.length) / (end.length - start.length);
        return (1 - s) * start.t + s * end.t;
    }

    template <Curve TCurve>
    void computeLength(const TCurve &curve) {
        if (lengthEstimationComplete()) {
            return;
        }

        Position pos(m_pos);
        while (pos.t < 1) {
            pos.advance(curve, m_step);
        }
        m_detachedlength = pos.length;
    }

    void markDegraded() {
        if (m_detachedlength == UNDEFINED) {
            m_detachedlength = DEGRADED;
        }
    }
    void clearDegraded() {
        if (m_detachedlength == DEGRADED || samplingComplete()) {
            m_detachedlength = UNDEFINED;
        }
    }

    double_t m_step;
    double_t m_tolerance;
    // INVARIANT: 0 <= m_t <= 1
    Position m_pos;
    // INVARIANT: 0 <= m_midt <= 1
    double_t m_midt;
    /// UDNEFINED, or DEGRADED, or the estimation of the length if it was detached from the estimation of `t` due to insufficient memory for sampling.
    double_t m_detachedlength;
    SmallDeque<Position> m_samples;
};

#endif