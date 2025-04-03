#ifndef _DEFINE_MATH_HPP_
#define _DEFINE_MATH_HPP_

#include <math.h> // type double_t

/* #region convert */

/** A phantom marker that specifies that lengths are expressed in meters. */
class Meter {
  public:
    /** Converts `value` from meters to `Unit` */
    template <typename Unit>
    static double_t convert(double_t value);
};

/** A phantom marker that specifies that lengths are expressed in millimeters. */
class Millimeter {
  public:
    /** Converts `value` from millimeters to `Unit` */
    template <typename Unit>
    static double_t convert(double_t value);
};

/** No-op */
template <>
inline double_t Meter::convert<Meter>(double_t value) {
    return value;
}
/** Converts `value` from meters to millimeters */
template <>
inline double_t Meter::convert<Millimeter>(double_t value) {
    return value * 1000.0;
}

/** No-op */
template <>
inline double_t Millimeter::convert<Millimeter>(double_t value) {
    return value;
}
/** Converts `value` from millimeters to meters */
template <>
inline double_t Millimeter::convert<Meter>(double_t value) {
    return value / 1000.0;
}

/* #endregion */

/// @return 0 if val == 0, 1 if val > 0, -1 if val < 0
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif