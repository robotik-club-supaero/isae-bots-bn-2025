#ifndef _BEZIER_HPP_
#define _BEZIER_HPP_

#include "geometry/Position2D.hpp"
#include "math/Polynomial.hpp"
#include <vector>

class BezierCurve {
  public:
    /// @param points There must be at least two points.
    BezierCurve(std::vector<Point2D<Meter>> points);
    BezierCurve(std::initializer_list<Point2D<Meter>> points);

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
     * Estimates the length of the curve using a discrete numerical method with the default number of steps.
     */
    double_t computeLength() const;

    /**
     * Estimates the length of the curve using a discrete numerical method.
     * Time complexity: O(num_steps).
     */
    double_t computeLength(unsigned int num_steps) const;

    const std::vector<Point2D<Meter>> &points() const;

  private:
    /// Pre-computed Bézier polynomial
    Polynomial<Point2D<Meter>> m_polynomial;
    Polynomial<Point2D<Meter>> m_derivative;
    std::vector<Point2D<Meter>> m_points;
};

#endif