#ifndef _POLYNOMIAL_HPP_
#define _POLYNOMIAL_HPP_

#include "defines/math.hpp"

#include <vector>

#ifdef _BR_DEBUG
#include <string>
#endif

/// A dense univariate polynomial.
///
/// @tparam T The type of the coefficients. Must be a complete type that implements addition with itself and multiplication with double.
template <Add T>
class Polynomial {

  public:
    /// Constructs the polynomial P(X) = 0.
    Polynomial() = default;

    /// @param coeffs coeffs[i] is the coefficient of degree i.
    Polynomial(std::vector<T> coeffs);
    /// @param coeffs The i-th element is the coefficient of degree i (where i is zero-based)
    Polynomial(std::initializer_list<T> coeffs);

    /// Evaluates the polynomial at `value` using Horner's method.
    T operator()(double_t value) const;

    Polynomial<T> derivative() const;

    const std::vector<T> &coeffs() const;
    std::vector<T> &coeffs();

#ifdef _BR_DEBUG
    operator std::string() const {
        std::string str = "(";
        for (int i = m_coeffs.size() - 1; i >= 0; i--) {
            str += std::string(m_coeffs[i]) + "X^" + std::to_string(i);
            if (i > 0) {
                str += ", ";
            }
        }
        str += ")";
        return str;
    }
#endif

  private:
    std::vector<T> m_coeffs;
};

#endif