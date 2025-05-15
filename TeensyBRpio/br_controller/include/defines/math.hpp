#ifndef _DEFINE_MATH_HPP_
#define _DEFINE_MATH_HPP_

#include "defines/constraint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using number_t = double;

template <typename T>
concept Add = requires(T a) {
    { a + a } -> std::convertible_to<T>;
    { a - a } -> std::convertible_to<T>;
};
template <typename TValue, typename TOutput = TValue>
concept Mul = requires(number_t a, TValue b) {
    { b *a } -> std::convertible_to<TOutput>;
};

/* #region convert */

/** A phantom marker that specifies that lengths are expressed in meters. */
class Meter {
  public:
    /** Converts `value` from meters to `Unit` */
    template <typename Unit>
    static number_t convert(number_t value);
};

/** A phantom marker that specifies that lengths are expressed in millimeters. */
class Millimeter {
  public:
    /** Converts `value` from millimeters to `Unit` */
    template <typename Unit>
    static number_t convert(number_t value);
};

/** No-op */
template <>
inline number_t Meter::convert<Meter>(number_t value) {
    return value;
}
/** Converts `value` from meters to millimeters */
template <>
inline number_t Meter::convert<Millimeter>(number_t value) {
    return value * 1000.0;
}

/** No-op */
template <>
inline number_t Millimeter::convert<Millimeter>(number_t value) {
    return value;
}
/** Converts `value` from millimeters to meters */
template <>
inline number_t Millimeter::convert<Meter>(number_t value) {
    return value / 1000.0;
}

template <typename From, typename To>
concept Convertible = requires(number_t a) {
    { From::template convert<To>(a) } -> std::convertible_to<number_t>;
};

/* #endregion */

/// @return 0 if val == 0, 1 if val > 0, -1 if val < 0
template <typename T>
constexpr inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

constexpr inline bool is_power_of_two(std::size_t n) {
    return n != 0 && (n & (n - 1)) == 0;
}

/* #region clamp */

/// Coordinate-wise clamping. See also std::clamp.
template <typename T>
T clamp(T value, number_t minBound, number_t maxBound);

template <typename T>
concept StdClampable = requires(T t, number_t b) {
    { std::clamp(t, b, b) } -> std::convertible_to<T>;
};

template <StdClampable T>
inline T clamp(T value, number_t minBound, number_t maxBound) {
    return std::clamp(value, minBound, maxBound);
}

template <typename T>
concept Clampable = requires(T t, number_t b) {
    { clamp<T>(t, -b, b) } -> std::convertible_to<T>;
};

/* #endregion */

#endif