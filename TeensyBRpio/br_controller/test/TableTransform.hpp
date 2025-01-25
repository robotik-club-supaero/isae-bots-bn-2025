#ifndef _TEST_TABLE_TRANSFORM_HPP_
#define _TEST_TABLE_TRANSFORM_HPP_

#include "geometry/Vector2D.hpp"

#include <SFML/Graphics.hpp>

/// Utility class to convert from physical (table) coordinates to screen coordinates (and vice versa).
class TableTransform {
  public:
    static constexpr double TABLE_WIDTH = 2;
    static constexpr double TABLE_HEIGHT = 3;

    static constexpr unsigned int WINDOW_WIDTH = 600;
    static constexpr unsigned int WINDOW_HEIGHT = 900;
    static constexpr unsigned int WINDOW_MARGIN = 20;

    sf::Vector2f toScreenCoordinates(Point2D<Meter> point) const {
        return sf::Vector2f(point.x * SCALE.x + WINDOW_MARGIN, WINDOW_HEIGHT + WINDOW_MARGIN - point.y * SCALE.y);
    }
    Point2D<Meter> toPhysicalCoordinates(int x, int y) const {
        return Point2D<Meter>((x - (int)WINDOW_MARGIN) / SCALE.x, ((int)(WINDOW_HEIGHT + WINDOW_MARGIN) - y) / SCALE.y);
    }

  private:
    static const Vector2D<Meter> SCALE;
};

const Vector2D<Meter> TableTransform::SCALE = {WINDOW_WIDTH / TABLE_WIDTH, WINDOW_HEIGHT / TABLE_HEIGHT};

#endif