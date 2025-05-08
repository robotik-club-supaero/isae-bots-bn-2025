#include "geometry/Bezier.hpp"
#include "logging.hpp"

/**
 * Iterator over (n choose i) for 0 <= i <= n.
 *
 * This leverages the fact that `(n i+1) = (n i) * (n - i) / (i + 1)` to compute each value in constant time
 * based on the previous value.
 *
 * Usage:
 * `for(BinomialIterator binom(n); binom.value; ++binom) {
 *      // (binom.n binom.i) == binom.value
 * }`
 *
 * Internal use only.
 */
struct BinomialIterator {

    unsigned int n;
    unsigned int i;
    unsigned int value;

    BinomialIterator(unsigned int n) : n(n), i(0), value(1) {}

    BinomialIterator &operator++() {
        value = (value * (n - i)) / (i + 1);
        i++;
        return *this;
    }

    BinomialIterator operator++(int) {
        BinomialIterator cpy = *this;
        ++(*this);
        return cpy;
    }

    operator unsigned int() const { return value; }
};

/**
 * Computes the polynomial `B` that corresponds to the Bézier curve defined by the given points.
 *
 * The curve is defined by: `B(t) = sum((n i) * t^i * (1 - t)^(n-i) * P_i)`, hence
 * `B(t) = sum[t^i *
 *                   (n i) * sum_{j<=i} ((-1)^(i-j) * (i j) * P_j)
 *         ]`.
 *
 * The second form allows to precompute the coefficients of the polynomial and can be evaluated using Horner's method.
 */
template <std::size_t N>
Polynomial<Point2D<Meter>, N> bezierToPolynomial(const std::array<Point2D<Meter>, N> &points) {
    std::array<Point2D<Meter>, N> coeffs;

    for (BinomialIterator binom(N - 1); binom.value; ++binom) {
        Point2D<Meter> coeff = {0, 0};
        for (BinomialIterator nested_binom(binom.i); nested_binom.value; ++nested_binom) {
            coeff += points[nested_binom.i] * ((int)nested_binom.value * (((binom.i + nested_binom.i) % 2) ? (-1) : 1));
        }
        coeffs[binom.i] = coeff * binom.value;
    }

    return coeffs;
}

template <std::size_t N>
BezierCurve<N>::BezierCurve(std::array<Point2D<Meter>, N> points)
    : m_polynomial(bezierToPolynomial(points)), m_derivative(m_polynomial.derivative()), m_secondDerivative(m_derivative.derivative()),
      m_points(std::move(points)) {}

template <std::size_t N>
Point2D<Meter> BezierCurve<N>::operator()(double_t t) const {
    return m_polynomial(t);
}

template <std::size_t N>
Vector2D<Meter> BezierCurve<N>::derivative(double_t t) const {
    return m_derivative(t);
}

template <std::size_t N>
double_t BezierCurve<N>::curvature(double_t t) const {
    Vector2D<Meter> derivative = m_derivative(t);
    double_t derivativeNorm = derivative.norm();
    Vector2D<Meter> secondDerivative = m_secondDerivative(t);

    return (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) / (derivativeNorm * derivativeNorm * derivativeNorm);
}

template <std::size_t N>
const std::array<Point2D<Meter>, N> &BezierCurve<N>::points() const {
    return m_points;
}

template <std::size_t N>
const Polynomial<Point2D<Meter>, N> &BezierCurve<N>::polynomial() const {
    return m_polynomial;
}

template class BezierCurve<4>;