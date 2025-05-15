#include "math/Integral.hpp"

#define TEMPLATE                                                                                                                                     \
    template <typename TValue>                                                                                                                       \
        requires Add<TValue> && Mul<TValue> && std::is_default_constructible_v<TValue> && Clampable<TValue>

TEMPLATE
Integral<TValue>::Integral(std::optional<number_t> saturation) : m_value(), m_saturation(saturation) {}

TEMPLATE
void Integral<TValue>::update(TValue value, number_t interval) {
    m_value += value * interval;
    if (m_saturation) {
        m_value = clamp<TValue>(m_value, -*m_saturation, *m_saturation);
    }
}

TEMPLATE
void Integral<TValue>::reset() {
    m_value = TValue();
}

TEMPLATE
Integral<TValue>::operator TValue() const {
    return m_value;
}

TEMPLATE
TValue Integral<TValue>::value() const {
    return m_value;
}

TEMPLATE
std::optional<number_t> Integral<TValue>::saturation() const {
    return m_saturation;
}

#include "geometry/Vector2D.hpp"
template class Integral<Vector2D<Meter>>;