#ifndef _BEZIER_HPP_
#define _BEZIER_HPP_

#include "geometry/Vector2D.hpp"
#include "math/Polynomial.hpp"

template <std::size_t N>
class BezierCurve {
  public:
    /// Constructs an empty Bézier curve, starting and ending at (0, 0).
    BezierCurve() = default;

    /// @param points The control points
    /// @{
    BezierCurve(std::array<Point2D<Meter>, N> points);

    template <typename... Args>
    BezierCurve(Args &&...points) : BezierCurve(std::array{std::forward<Args>(points)...}) {}
    /// @}

    /**
     * Returns `B(t)` where `B` is the function describing this Bézier curve.
     * @param t The position on the curve. Must be between 0 and 1 (both included), otherwise the result is unspecified.
     */
    Point2D<Meter> operator()(double_t t) const;

    /**
     * Computes the derivative `dB(t)/dt` where `B` is the function describing this Bézier curve.
     * @param t The position on the curve. Must be between 0 and 1 (both included), otherwise the result is unspecified.
     */
    Vector2D<Meter> derivative(double_t t) const;

    /**
     * Computes the *signed* curvature of the curve. This is the reciprocal of the radius of curvature.
     * @param t The position on the curve. Must be between 0 and 1 (both included), otherwise the result is unspecified.
     */
    double_t curvature(double_t t) const;

    const std::array<Point2D<Meter>, N> &points() const;

    const Polynomial<Point2D<Meter>, N> &polynomial() const;

  private:
    /// Pre-computed Bézier polynomial
    Polynomial<Point2D<Meter>, N> m_polynomial;
    Polynomial<Point2D<Meter>, N>::Derivative m_derivative;
    Polynomial<Point2D<Meter>, N>::Derivative::Derivative m_secondDerivative;
    std::array<Point2D<Meter>, N> m_points;
};

#endif