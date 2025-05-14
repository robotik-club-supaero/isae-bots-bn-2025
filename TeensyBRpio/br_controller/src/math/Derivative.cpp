#include "math/Derivative.hpp"

#define TEMPLATE                                                                                                                                     \
    template <typename TDerivative, typename TValue>                                                                                                 \
        requires Add<TValue> && Mul<TValue, TDerivative> && std::is_default_constructible_v<TValue> && Clampable<TValue>

TEMPLATE
Derivative<TDerivative, TValue>::Derivative(double_t gain, double_t filter, TValue initialValue, std::optional<double_t> saturation)
    : m_gain(gain), m_filter(filter), m_lastValue(initialValue), m_lastDerivative(), m_saturation(saturation) {}

TEMPLATE
void Derivative<TDerivative, TValue>::update(TValue value, double_t interval) {
    m_lastDerivative = m_gain / (interval + m_filter * m_gain) * (m_lastDerivative * m_filter + (value - m_lastValue));
    if (m_saturation) {
        m_lastDerivative = clamp<TValue>(m_lastDerivative, -*m_saturation, *m_saturation);
    }
    m_lastValue = value;
}

TEMPLATE
void Derivative<TDerivative, TValue>::reset(TValue initialValue) {
    m_lastValue = initialValue;
    m_lastDerivative = TValue();
}

TEMPLATE
Derivative<TDerivative, TValue>::operator TDerivative() const {
    return m_lastDerivative;
}

TEMPLATE
TDerivative Derivative<TDerivative, TValue>::value() const {
    return m_lastDerivative;
}

TEMPLATE
TValue Derivative<TDerivative, TValue>::getLastInput() const {
    return m_lastValue;
}

TEMPLATE
double_t Derivative<TDerivative, TValue>::gain() const {
    return m_gain;
}

TEMPLATE
double_t Derivative<TDerivative, TValue>::filter() const {
    return m_filter;
}

TEMPLATE
std::optional<double_t> Derivative<TDerivative, TValue>::saturation() const {
    return m_saturation;
}

#include "geometry/Position2D.hpp"
template class Derivative<Vector2D<Meter>>;
template class Derivative<Position2D<Meter, double_t>, Position2D<Meter>>;