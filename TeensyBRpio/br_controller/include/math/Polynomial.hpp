#ifndef _POLYNOMIAL_HPP_
#define _POLYNOMIAL_HPP_

#include "defines/math.hpp"

#include <array>

#ifdef _BR_DEBUG
#include <string>
#endif

template <std::size_t N>
struct DerivativeDegree {
    static constexpr std::size_t value = N - 1;
};
template <>
struct DerivativeDegree<0> {
    static constexpr std::size_t value = 0;
};

/// A dense univariate polynomial.
///
/// @tparam T The type of the coefficients. Must be a complete type that implements addition with itself and multiplication with double.
template <Add T, std::size_t N>
class Polynomial {

  public:
    using Derivative = Polynomial<T, DerivativeDegree<N>::value>;

    /// Constructs the polynomial P(X) = 0.
    Polynomial() = default;

    /// @param coeffs coeffs[i] is the coefficient of degree i.
    Polynomial(std::array<T, N> coeffs);

    /// Evaluates the polynomial at `value` using Horner's method.
    T operator()(double_t value) const;

    Derivative derivative() const;

    const std::array<T, N> &coeffs() const;
    std::array<T, N> &coeffs();

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
    std::array<T, N> m_coeffs;
};

#endif