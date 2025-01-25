#ifndef _TEST_WINDOW_HPP_

#include "TableTransform.hpp"
#include "geometry/Vector2D.hpp"

#include <SFML/Graphics.hpp>

class Window : public TableTransform, public sf::RenderWindow {
  public:
    Window(const sf::String &name) : RenderWindow(sf::VideoMode(WINDOW_WIDTH + 2 * WINDOW_MARGIN, WINDOW_HEIGHT + 2 * WINDOW_MARGIN), name) {}
};

#endif