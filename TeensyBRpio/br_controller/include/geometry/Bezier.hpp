#ifndef _BEZIER_HPP_
#define _BEZIER_HPP_

#include "geometry/Vector2D.hpp"
#include "math/Polynomial.hpp"
#include <vector>

class BezierCurve {
  public:

    /// Constructs an empty Bézier curve, starting and ending at (0, 0).
    BezierCurve() = default;

    /// @param points The control points
    /// @{
    BezierCurve(std::vector<Point2D<Meter>> points);
    BezierCurve(std::initializer_list<Point2D<Meter>> points);
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

    const std::vector<Point2D<Meter>> &points() const;

    const Polynomial<Point2D<Meter>> &polynomial() const;

  private:
    /// Pre-computed Bézier polynomial
    Polynomial<Point2D<Meter>> m_polynomial;
    Polynomial<Point2D<Meter>> m_derivative;
    Polynomial<Point2D<Meter>> m_secondDerivative;
    std::vector<Point2D<Meter>> m_points;
};

#endif