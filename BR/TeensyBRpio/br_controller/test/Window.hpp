#ifndef _TEST_WINDOW_HPP_

#include "geometry/Vector2D.hpp"
#include <SFML/Graphics.hpp>

class Window: public sf::RenderWindow {
  public:
    Window(const sf::String &name)
        : RenderWindow(sf::VideoMode(WINDOW_WIDTH + 2 * WINDOW_MARGIN, WINDOW_HEIGHT + 2 * WINDOW_MARGIN), name),
          m_scale(WINDOW_WIDTH / TABLE_WIDTH, WINDOW_HEIGHT / TABLE_HEIGHT) {}

    sf::Vector2f toScreenCoordinates(Point2D<Meter> point) const {
        return sf::Vector2f(point.x * m_scale.x + WINDOW_MARGIN, WINDOW_HEIGHT + WINDOW_MARGIN - point.y * m_scale.y);
    }

  private:
    static constexpr double TABLE_WIDTH = 2;
    static constexpr double TABLE_HEIGHT = 3;

    static constexpr unsigned int WINDOW_WIDTH = 600;
    static constexpr unsigned int WINDOW_HEIGHT = 900;
    static constexpr unsigned int WINDOW_MARGIN = 20;

    Vector2D<Meter> m_scale;
};

#endif