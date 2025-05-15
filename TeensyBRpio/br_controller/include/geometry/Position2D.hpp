#ifndef _POSITION_2D_HPP_
#define _POSITION_2D_HPP_

#include "defines/math.hpp"
#include "defines/string.hpp"
#include "geometry/Angle.hpp"
#include "geometry/Speeds.hpp"
#include "geometry/Vector2D.hpp"

/**
 * A 2D-position with orientation. The unit of x and y is given by type parameter `Unit`.
 * theta is in radians.
 *
 * @tparam Unit The unit of x and y. Only `Meter` and `Millimeter` are supported.
 * @tparam TAngle The type of the angle. Only `Angle` and `number_t` are supported.
 */
template <typename Unit, typename TAngle = Angle>
class Position2D : public Vector2D<Unit> {
  public:
    Position2D();
    Position2D(number_t x, number_t y, TAngle theta);
    Position2D(Vector2D<Unit> position, TAngle theta = 0);

    template <typename To>
    operator Position2D<Unit, To>() const {
        return Position2D<Unit, To>(this->x, this->y, static_cast<To>(theta));
    }

    template <typename To>
        requires Convertible<Unit, To>
    Position2D<To, TAngle> convert() const {
        return Position2D<To, TAngle>(Vector2D<Unit>::template convert<To>(), theta);
    }

    Position2D<Millimeter, TAngle> toMillimeters() const
        requires Convertible<Unit, Millimeter>
    {
        return convert<Millimeter>();
    }
    Position2D<Meter, TAngle> toMeters() const
        requires Convertible<Unit, Meter>
    {
        return convert<Meter>();
    }

    /**
     * Returns this position with the orientation reversed.
     */
    Position2D<Unit, Angle> flip() const;

    /**
     * Returns the absolute coordinates of `vector`.
     * @param vector The relative coordinates of the vector in relation to this position.
     */
    Vector2D<Unit> makeAbsolute(Vector2D<Unit> vector) const;

    /**
     * Returns the relative coordinates of `vector` in relation to this position.
     * @param vector The absolute coordinates of the vector, in the same global frame as this position.
     */
    Vector2D<Unit> makeRelative(Vector2D<Unit> vector) const;

    /**
     * Returns the new position after applying the specified relative offset.
     * @param xr The signed distance to move in the direction of this position's theta.
     * @param yr The signed distance to move in the direction normal to this position's theta.
     * @param theta The signed angle to rotate
     *
     * This is equivalent to `this + Position2D(this.makeAbsolute({xr, yr}), theta)`.
     */
    Position2D<Unit, TAngle> relativeOffset(number_t xr, number_t yr = 0, TAngle theta = 0) const;

    void operator+=(Position2D<Unit, TAngle> pos);
    void operator-=(Position2D<Unit, TAngle> pos);

    void operator+=(Speeds relativeOffset);
    void operator-=(Speeds relativeOffset);

    bool operator==(const Position2D<Unit, TAngle> &pos) const = default;

    Position2D<Unit, TAngle> operator+(Position2D<Unit, Angle> pos) const;
    Position2D<Unit, TAngle> operator-(Position2D<Unit, Angle> pos) const;
    Position2D<Unit, number_t> operator+(Position2D<Unit, number_t> pos) const;
    Position2D<Unit, number_t> operator-(Position2D<Unit, number_t> pos) const;

    Position2D<Unit, TAngle> operator+(Speeds relativeOffset) const;
    Position2D<Unit, TAngle> operator-(Speeds relativeOffset) const;

    Position2D<Unit, number_t> operator*(number_t factor) const;
    Position2D<Unit, number_t> operator/(number_t factor) const;
    Position2D<Unit, number_t> operator*(Speeds factor) const;
    Position2D<Unit, number_t> operator/(Speeds factor) const;

    TAngle theta;

#ifdef _STRING_EXT_
    operator std::string() const { return "(" + to_string(this->x) + ", " + to_string(this->y) + ", " + to_string(theta) + ")"; }
#endif
};

template <typename Unit, typename TAngle>
inline Position2D<Unit, number_t> operator*(number_t factor, Position2D<Unit, TAngle> pos) {
    return pos * factor;
}
template <typename Unit, typename TAngle>
inline Position2D<Unit, number_t> operator*(Speeds factor, Position2D<Unit, TAngle> pos) {
    return pos * factor;
}

#endif