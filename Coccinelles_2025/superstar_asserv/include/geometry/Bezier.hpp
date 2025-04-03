#ifndef _BEZIER_HPP_
#define _BEZIER_HPP_

#include "geometry/Position2D.hpp"
#include "math/Polynomial.hpp"
#include "math/Samples.hpp"
#include <vector>

class BezierCurve {
  public:
    /// @param points There must be at least two points.
    /// @{
    BezierCurve(std::vector<Point2D<Meter>> points);
    BezierCurve(std::initializer_list<Point2D<Meter>> points);
    /// @}

    struct LengthSamples {
        double_t totalLength;
        /// Maps distance from start (in meters) to the curve parameter (t between 0 and 1)
        Samples<> samples;     
    };

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

    /**
     * Calls `sampleLength(double_t)` with the default step size.
     * @see BezierCurve::sampleLength(double_t)`.
     */
    LengthSamples sampleLength() const;

    /**
     * Walks the curve in steps of (approximately) `step` meters to estimate the length of the curve. Although `step` determines the
     * accuracy of the estimation, the estimation error may be smaller than `step`.
     *
     * @return The estimated length of the curve (denoted `L`) and the samples of function `t(l)` that converts the distance from the
     * start of the curve to the curve parameter (i.e. B(t(l)) is the position on the curve after walking `l` meters from the start,
     * i.e. Integral_0^{t(l)} || B'(x) dx || = l).
     * 
     * Time and space complexity is linear with respect to `L / step`.
     */
    LengthSamples sampleLength(double_t step) const;

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