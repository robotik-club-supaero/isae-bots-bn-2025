#ifndef _CURVE_SAMPLING_HPP_
#define _CURVE_SAMPLING_HPP_

#include "geometry/Vector2D.hpp"
#include "stl/SmallDeque.hpp"

#include <concepts>

/// A differentiable parametric curve
template <typename T>
concept Curve = requires(const T &c, number_t t) {
    { c(t) } -> std::convertible_to<Point2D<Meter>>;
    { c.derivative(t) } -> std::convertible_to<Vector2D<Meter>>;
    { c.curvature(t) } -> std::convertible_to<number_t>;
};

/// Adaptive yet robust curve sampling suitable for low memory environments.
///
/// This class implements:
/// - length estimation
/// - amortized constant-time inverse arc length estimation (parameter t as a function of the distance).
///
/// Memory usage only depends on the complexity of the curve, not its length. Perfect lines don't require any dynamic allocation at all.
/// Worst-case time complexity is O(length/step) which occurs when sampling is required - either because it hasn't been performed yet, or because
/// previously sampled values could not be stored due to insufficient memory.
class CurveSampling {
  public:
    CurveSampling(number_t step) : CurveSampling(step, step / 2) {}
    CurveSampling(number_t step, number_t tolerance) : m_step(step), m_tolerance(tolerance), m_pos(), m_mid_pos(), m_degradedPos(), m_samples() {}

    /// Samples the curve over the specified distance (starting from the beginning of the curve). If the specified distance was already sampled, this
    /// is a no-op.
    ///
    /// The behavior is undefined if `dist < 0` or `curve` is different from the curves used in previous calls to `sample` or `solveInverseArcLength`.
    ///
    /// Sampling might be performed in a degraded mode if the memory is insufficient. In this case, `isDegraded()` will be set to true and
    /// `degradedAfter()` can be used to check where the sampling began to be degraded. When the memory is sufficient again, `sample` can be called
    /// again to solve the problem.
    template <Curve T>
    void sample(const T &curve, number_t dist) {
        number_t last_sample_t = m_samples.empty() ? 0 : m_samples.back().t;

        while (m_pos.t < 1 && m_pos.length < dist) {
            if (m_pos.t > 0 && m_mid_pos.isAdvanceStep()) {
                number_t midt_est = (last_sample_t + m_pos.t) / 2;
                if (std::abs(m_mid_pos - midt_est) > m_tolerance) {
                    if (m_samples.push_back(m_pos)) {
                        last_sample_t = m_pos.t;
                        m_mid_pos = m_pos.t;
                    } else {
                        if (!isDegraded()) {
                            log(WARN, "Curve sampling is running short on memory. Falling back to degraded sampling.");
                        }
                        m_degradedPos = m_pos;
                        sampleDegraded(curve, dist);
                        return;
                    }
                }
            }

            if (!m_pos.advance(curve, m_step)) {
                log(ERROR, "Aborting sampling of zero-length or ill-formed curve");
                break;
            }

            m_mid_pos.advanceMaybe(curve, m_step);
        }
    }

    /// Whether part of the sampling is "degraded".
    ///
    /// This means that some of the samples necessary to solve the inverse arc length could not be stored because the memory was insufficient.
    bool isDegraded() const { return m_pos.t < 1 && m_degradedPos.length > m_pos.length; }

    /// Allows to discard any sample that is not useful to solve the inverse arc length for `dist >= before`.
    ///
    /// If, afterwards, `solveInverseArcLength` is called with a distance less than `before`, the behavior is undefined.
    void discardSamples(number_t before) {
        if (!m_samples.empty() && m_samples.front().length < before) {
            // If sample0 < sample1 < before < sample2, we can drop sample0 but we must keep sample1 because it affects estimation of `dist` in
            // [before, sample2].
            while (m_samples.size() >= 2 && m_samples[1].length < before) {
                m_samples.pop_front();
            }
        }
    }

    bool samplingComplete() const { return m_pos.t == 1 || m_degradedPos.t == 1; }

    /// The estimated length of the curve. If `samplingComplete()` is `false`, the result is unspecified.
    number_t length() const {
        if (samplingComplete()) {
            return m_pos.length;
        } else {
            return m_degradedPos.length;
        }
    }

    number_t sampledLength() const {
        if (isDegraded()) {
            return m_degradedPos.length;
        } else {
            return m_pos.length;
        }
    }

    /// Returns the distance (from the beginning of the trajectory) after which the sampling was performed in a degraded mode.
    ///
    /// The estimation of the inverse arc length may lack accuracy when `dist > degradedAfter()`, especially if
    /// `sampledLength()` is much larger than `degradedAfter()`. Sampling will try to resume from this distance when `sample` is called again.
    ///
    /// If `isDegraded()` is false, the result is unspecified.
    number_t degradedAfter() const { return m_pos.length; }

    /// Estimates the parameter `t` such that `Integral_[0, t] ||C'(l)|| dl = dist`, where `C` is the curve that was sampled.
    ///
    /// The behavior is undefined if `dist < 0` or `dist > sampledLength()`. The result is guaranteed to be consistent accross calls and
    /// non-decreasing only if `isDegraded()` is false or `dist <= degradedAfter()`.
    number_t solveInverseArcLength(number_t dist) const {
        if (!samplingComplete() && dist > m_pos.length) {
            return interpolate(dist, m_pos, m_degradedPos);
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
    /// If `isDegraded()` is true and `degradedAfter() < dist` when this function returns, the result may lack accuracy and consistency.
    /// See `sample()`, `isDegraded()` and `degradedAfter()` for more information.
    template <Curve T>
    number_t solveInverseArcLength(const T &curve, number_t dist) {
        sample(curve, dist);
        return solveInverseArcLength(dist);
    }

    std::size_t numberOfSamplePoints() const { return m_samples.size(); }

  private:
    struct MidPosition {
        constexpr MidPosition() = default;
        constexpr MidPosition(number_t t) : t(t) {}
        constexpr operator number_t() const { return std::abs(t); }

        // INVARIANT: 0 <= |m_midt| <= 1. The sign bit is used to track even and odd steps (so we advance m_midt only every second step).
        number_t t;

        template <Curve TCurve>
        void advanceMaybe(const TCurve &curve, number_t dist) {
            toggleStep();
            if (isAdvanceStep()) {
                t += dist / curve.derivative(t).norm();
            }
        }

        bool isAdvanceStep() {
            // Positive => even step (should advance mid pos)
            // Negative => odd step
            return !std::signbit(t);
        }

      private:
        void toggleStep() {
            if (std::signbit(t)) {
                t = std::copysign(t, 1);
            } else {
                t = std::copysign(t, -1);
            }
        }
    };

    struct Position {
        constexpr Position() = default;
        constexpr Position(number_t length, number_t t) : length(length), t(t) {}
        number_t length;
        number_t t;

        template <Curve TCurve>
        bool advance(const TCurve &curve, number_t dist) {
            number_t derivative = curve.derivative(t).norm();
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

    static number_t interpolate(number_t dist, const Position &start, const Position &end) {
        if (end.length == start.length || dist <= start.length) {
            return start.t;
        }
        if (end.length <= dist) {
            return end.t;
        }

        number_t s = (dist - start.length) / (end.length - start.length);
        return (1 - s) * start.t + s * end.t;
    }

    /// Samples from `m_degradedPos` to `dist` but discards all the samples that would be necessary to accurately solve the inverse arc length.
    ///
    /// This only advances `m_degradedPos`, so normal sampling may be resumed at any time.
    ///
    /// If `m_degradedPos` is strictly before `m_pos`, the behavior is undefined.
    template <Curve TCurve>
    void sampleDegraded(const TCurve &curve, number_t dist) {
        if (samplingComplete()) {
            return;
        }

        while (m_degradedPos.t < 1 && m_degradedPos.length < dist) {
            m_degradedPos.advance(curve, m_step);
        }
    }

    number_t m_step;
    number_t m_tolerance;
    // Position of the normal sampling
    Position m_pos;
    // Value of `t` at `
    MidPosition m_mid_pos;
    /// Position of the "degraded" sampling. If this is before `m_pos`, the sampling is not degraded and this must not relied upon.
    Position m_degradedPos;
    SmallDeque<Position> m_samples;
};

#endif