#ifndef _POLYNOMIAL_HPP_
#define _POLYNOMIAL_HPP_

#include "defines/math.hpp"

#include <vector>

#ifdef _DEBUG
#include <string>
#endif

template <Add T>
class Polynomial {

  public:
    /// @param coeffs coeffs[i] is the coefficient of degree i.
    Polynomial(std::vector<T> coeffs);
    /// @param coeffs The i-th element is the coefficient of degree i (where i is zero-based)
    Polynomial(std::initializer_list<T> coeffs);

    /// Evaluates the polynomial at `value` using Horner's method.
    T operator()(double_t value) const;

    Polynomial<T> derivative() const;

    const std::vector<T> &coeffs() const;
    std::vector<T> &coeffs();

#ifdef _DEBUG
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