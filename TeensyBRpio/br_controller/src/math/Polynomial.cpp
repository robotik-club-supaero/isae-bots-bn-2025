#include "math/Polynomial.hpp"

template <Add T>
Polynomial<T>::Polynomial(std::vector<T> coeffs) : m_coeffs(std::move(coeffs)) {}
template <Add T>
Polynomial<T>::Polynomial(std::initializer_list<T> coeffs) : Polynomial(std::vector<T>(coeffs)) {}

template <Add T>
T Polynomial<T>::operator()(double_t value) const {
    T r;
    for (auto coeff = m_coeffs.rbegin(); coeff != m_coeffs.rend(); coeff++) {
        r = r * value + *coeff;
    }
    return r;
}

template <Add T>
Polynomial<T> Polynomial<T>::derivative() const {
    std::vector<T> coeffs;
    for (auto i = 1; i < m_coeffs.size(); i++) {
        coeffs.push_back(i * m_coeffs[i]);
    }
    return Polynomial(std::move(coeffs));
}

template <Add T>
const std::vector<T> &Polynomial<T>::coeffs() const {
    return m_coeffs;
}

template <Add T>
std::vector<T> &Polynomial<T>::coeffs() {
    return m_coeffs;
}
#include "geometry/Vector2D.hpp"
template class Polynomial<Point2D<Meter>>;