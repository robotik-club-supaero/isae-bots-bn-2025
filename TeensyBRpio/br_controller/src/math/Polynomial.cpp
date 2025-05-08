#include "math/Polynomial.hpp"

template <Add T, std::size_t N>
Polynomial<T, N>::Polynomial(std::array<T, N> coeffs) : m_coeffs(std::move(coeffs)) {}

template <Add T, std::size_t N>
T Polynomial<T, N>::operator()(double_t value) const {
    T r;
    for (auto coeff = m_coeffs.rbegin(); coeff != m_coeffs.rend(); coeff++) {
        r = r * value + *coeff;
    }
    return r;
}

template <Add T, std::size_t N>
Polynomial<T, N>::Derivative Polynomial<T, N>::derivative() const {
    std::array<T, DerivativeDegree<N>::value> coeffs;
    for (unsigned int i = 1; i < N; i++) {
        coeffs[i-1] = i * m_coeffs[i];
    }
    return coeffs;
}

template <Add T, std::size_t N>
const std::array<T, N> &Polynomial<T, N>::coeffs() const {
    return m_coeffs;
}

template <Add T, std::size_t N>
std::array<T, N> &Polynomial<T, N>::coeffs() {
    return m_coeffs;
}

#include "geometry/Vector2D.hpp"
template class Polynomial<Point2D<Meter>, 4>;
template class Polynomial<Point2D<Meter>, 3>;
template class Polynomial<Point2D<Meter>, 2>;
template class Polynomial<Point2D<Meter>, 1>;
template class Polynomial<Point2D<Meter>, 0>;