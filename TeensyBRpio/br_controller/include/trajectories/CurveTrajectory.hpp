#ifndef _CURVE_TRAJECTORY_HPP_
#define _CURVE_TRAJECTORY_HPP_

#include "logging.hpp"
#include "math/RollingMax.hpp"
#include "trajectories/Trajectory.hpp"

#include <concepts>

/// A differentiable parametric curve
template <typename T>
concept Curve = requires(const T &c, double_t t) {
    { c(t) } -> std::convertible_to<Point2D<Meter>>;
    { c.derivative(t) } -> std::convertible_to<Vector2D<Meter>>;
    { c.curvature(t) } -> std::convertible_to<double_t>;
};

/// A trajectory that follows a differentiable parametric curve.
/// @tparam TCurve The type of the trajectory. Must be a complete type.
template <Curve TCurve>
class CurveTrajectory : public Trajectory {
  public:
    /**
     * @param curve
     * @param samplingStep The step (in meter) used to estimate the length of the trajectory and sample the curvature
     * @param advanceMaxStep The maximum integration step (in meter) used in `advance()`. If the requested distance is greater,
     * it will be split in multiple steps less than or equal to the maximum step.
     */
    CurveTrajectory(TCurve curve, double_t samplingStep = 0.005, double_t advanceMaxStep = 0.005)
        : m_curve(std::move(curve)), m_length(0), m_curvature(), m_advanceMaxStep(advanceMaxStep), m_currentDerivative(m_curve.derivative(0)),
          m_distance(0), m_t(0) {
        std::vector<double_t> curvatures;
        double_t t = 0;
        double_t derivative;
        while (t <= 1) {
            curvatures.push_back(abs(m_curve.curvature(t)));

            derivative = m_curve.derivative(t).norm();
            if (derivative == 0) {
                log(ERROR, "Zero-length or ill-formed curve given to CurveTrajectory.");
                m_t = 1;
                t = 1;
                break;
            }

            t += samplingStep / derivative;
        }
        if (t > 1) {
            // HACK: mapping m_curve.curvature(1) to curvatures(t) to force sample edges of curve
            curvatures.push_back(abs(m_curve.curvature(1)));
        }

        double_t domainEnd = samplingStep * (curvatures.size() - 1);
        m_curvature = RollingMax(Samples(std::move(curvatures), 0, domainEnd));

        m_length = domainEnd + (1 - t) * derivative;
    }

    template <typename... Args>
        requires std::is_constructible<TCurve, Args...>::value
    CurveTrajectory(Args &&...args) : CurveTrajectory(TCurve(std::forward<Args>(args)...)) {}

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override {
        if (m_t >= 1) {
            return false;
        }

        m_distance += distance;
        while (m_t < 1 && distance > 0) {
            m_t += std::min(distance, m_advanceMaxStep) / m_currentDerivative.norm();
            distance -= m_advanceMaxStep;
            if (m_t >= 1 || m_distance >= m_length) {
                m_distance = m_length;
                m_t = 1;
            }
            m_currentDerivative = m_curve.derivative(m_t);
        }

        return true;
    }

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override { return Position2D<Meter>(m_curve(m_t), m_currentDerivative.argument()); }

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override { return m_length - m_distance; }

    /// @copydoc Trajectory::getMaxCurvature()
    double_t getMaxCurvature(double_t distance) const override {
        return m_curvature.getMaximum(std::min(m_curvature.domainEnd(), m_distance), std::min(m_curvature.domainEnd(), m_distance + distance));
    }

    const TCurve &getCurve() const { return m_curve; }

  private:
    TCurve m_curve;
    double_t m_length;
    RollingMax m_curvature;
    double_t m_advanceMaxStep;

    Vector2D<Meter> m_currentDerivative;
    double_t m_distance;
    double_t m_t;
};

#endif