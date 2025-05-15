#include "geometry/Position2D.hpp"
#include <cmath>

template <typename Unit, typename TAngle>
Position2D<Unit, TAngle>::Position2D() : Vector2D<Unit>(), theta(0) {}

template <typename Unit, typename TAngle>
Position2D<Unit, TAngle>::Position2D(number_t x, number_t y, TAngle theta) : Vector2D<Unit>(x, y), theta(theta) {}

template <typename Unit, typename TAngle>
Position2D<Unit, TAngle>::Position2D(Vector2D<Unit> pos, TAngle theta) : Vector2D<Unit>(pos), theta(theta) {}

template <typename Unit, typename TAngle>
Position2D<Unit, Angle> Position2D<Unit, TAngle>::flip() const {
    return Position2D<Unit, Angle>(this->x, this->y, Angle(theta).reverse());
}

template <typename Unit, typename TAngle>
Vector2D<Unit> Position2D<Unit, TAngle>::makeAbsolute(Vector2D<Unit> vector) const {
    number_t cos = std::cos(theta);
    number_t sin = std::sin(theta);

    return Vector2D<Unit>(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
}

template <typename Unit, typename TAngle>
Vector2D<Unit> Position2D<Unit, TAngle>::makeRelative(Vector2D<Unit> vector) const {
    return Position2D<Unit, TAngle>(this->x, this->y, -theta).makeAbsolute(vector);
}

template <typename Unit, typename TAngle>
Position2D<Unit, TAngle> Position2D<Unit, TAngle>::relativeOffset(number_t xr, number_t yr, TAngle theta) const {
    Vector2D<Unit> vector = makeAbsolute(Vector2D<Unit>(xr, yr));

    return Position2D<Unit, TAngle>(this->x + vector.x, this->y + vector.y, this->theta + theta);
}

template <typename Unit, typename TAngle>
void Position2D<Unit, TAngle>::operator+=(Position2D<Unit, TAngle> pos) {
    Vector2D<Unit>::operator+=(pos);
    theta += pos.theta;
}
template <typename Unit, typename TAngle>
void Position2D<Unit, TAngle>::operator-=(Position2D<Unit, TAngle> pos) {
    Vector2D<Unit>::operator-=(pos);
    theta -= pos.theta;
}

template <typename Unit, typename TAngle>
void Position2D<Unit, TAngle>::operator+=(Speeds offset) {
    *this = *this + offset;
}
template <typename Unit, typename TAngle>
void Position2D<Unit, TAngle>::operator-=(Speeds offset) {
    *this = *this - offset;
}

template <typename Unit, typename TAngle>
Position2D<Unit, TAngle> Position2D<Unit, TAngle>::operator+(Position2D<Unit, Angle> pos) const {
    return Position2D<Unit, TAngle>(Vector2D<Unit>::operator+(pos), theta + static_cast<TAngle>(pos.theta));
}
template <typename Unit, typename TAngle>
Position2D<Unit, TAngle> Position2D<Unit, TAngle>::operator-(Position2D<Unit, Angle> pos) const {
    return Position2D<Unit, TAngle>(Vector2D<Unit>::operator-(pos), theta - static_cast<TAngle>(pos.theta));
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator+(Position2D<Unit, number_t> pos) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator+(pos), static_cast<number_t>(theta) + pos.theta);
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator-(Position2D<Unit, number_t> pos) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator-(pos), static_cast<number_t>(theta) - pos.theta);
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator*(number_t factor) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator*(factor), theta * factor);
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator/(number_t factor) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator/(factor), theta / factor);
}
template <typename Unit, typename TAngle>
Position2D<Unit, TAngle> Position2D<Unit, TAngle>::operator+(Speeds offset) const {
    return relativeOffset(offset.linear, 0, offset.angular);
}
template <typename Unit, typename TAngle>
Position2D<Unit, TAngle> Position2D<Unit, TAngle>::operator-(Speeds offset) const {
    return *this + (-offset);
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator*(Speeds factor) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator*(factor.linear), theta * factor.angular);
}
template <typename Unit, typename TAngle>
Position2D<Unit, number_t> Position2D<Unit, TAngle>::operator/(Speeds factor) const {
    return Position2D<Unit, number_t>(Vector2D<Unit>::operator/(factor.linear), theta / factor.angular);
}

#define __SPECIALIZE(UNIT, ANGLE)                                                                                                                    \
    template class Position2D<UNIT, ANGLE>;                                                                                                          \
                                                                                                                                                     \
    template <>                                                                                                                                      \
    Position2D<UNIT, ANGLE> clamp<Position2D<UNIT, ANGLE>>(Position2D<UNIT, ANGLE> value, number_t minBound, number_t maxBound) {                    \
        return Position2D<UNIT, ANGLE>(clamp<Vector2D<UNIT>>(value, minBound, maxBound), value.theta);                                               \
    }

__SPECIALIZE(Meter, Angle);
__SPECIALIZE(Meter, number_t);
__SPECIALIZE(Millimeter, Angle);
__SPECIALIZE(Millimeter, number_t);

#undef __SPECIALIZE
