#ifndef _TEST_JOYSTICK_HPP_
#define _TEST_JOYSTICK_HPP_

#include <SFML/Graphics.hpp>

/// A joystick for mouse control
class Joystick : public sf::Drawable {
  public:
    /// @param screenPosition Position of the top-left corner of the joystick's bounding box.
    Joystick(sf::Vector2f screenPosition)
        : m_position(screenPosition + sf::Vector2f(BACKGROUND_SIZE, BACKGROUND_SIZE)), m_backgroundMarker(BACKGROUND_SIZE), m_headMarker(HEAD_SIZE),
          m_pressed(), m_value() {
        m_backgroundMarker.setPosition(m_position);
        m_backgroundMarker.setOrigin(BACKGROUND_SIZE, BACKGROUND_SIZE);
        m_backgroundMarker.setFillColor(sf::Color::White);

        m_headMarker.setFillColor(sf::Color::Black);
        setValue({0, 0});
    }

    bool processEvent(const sf::Event &event) {
        switch (event.type) {
            case sf::Event::MouseButtonPressed:
                if (event.mouseButton.button == sf::Mouse::Left) {
                    float x = event.mouseButton.x - m_position.x;
                    float y = event.mouseButton.y - m_position.y;
                    float norm = sqrt(x * x + y * y) / BACKGROUND_SIZE;
                    if (norm <= 1) {
                        m_pressed = true;
                        setValueFromMouse(event.mouseButton);
                        return true;
                    }
                }
                break;
            case sf::Event::MouseMoved:
                if (m_pressed) {
                    setValueFromMouse(event.mouseMove);
                    return true;
                } else {
                    return false;
                }
            case sf::Event::MouseButtonReleased:
                if (event.mouseButton.button == sf::Mouse::Left) {
                    m_pressed = false;
                    setValue({0, 0});
                    return true;
                }
                break;
        }
        return false;
    }

    /// Returns the displacement of the joystick's head (x and y are between -1 and 1).
    sf::Vector2f getValue() const { return m_value; }

    /// Sets the displacement of the joystick's head (x and y should be between -1 and 1).
    void setValue(sf::Vector2f value) {
        float norm = sqrt(value.x * value.x + value.y * value.y);
        if (norm > 1) {
            value /= norm;
        }
        m_value = value;

        m_headMarker.setPosition(m_position - sf::Vector2f(HEAD_SIZE, HEAD_SIZE) + (float)BACKGROUND_SIZE * m_value);
    }

  protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override {
        target.draw(m_backgroundMarker, states);
        target.draw(m_headMarker, states);
    }

  private:
    template <typename T>
    void setValueFromMouse(T event) {
        float x = event.x - m_position.x;
        float y = event.y - m_position.y;
        setValue(sf::Vector2f(x, y) / (float)BACKGROUND_SIZE);
    }

    sf::Vector2f m_position;
    sf::CircleShape m_backgroundMarker;
    sf::CircleShape m_headMarker;

    bool m_pressed;
    sf::Vector2f m_value;

    static constexpr unsigned int BACKGROUND_SIZE = 100;
    static constexpr unsigned int HEAD_SIZE = 10;
};

#endif