#ifndef _TEST_POINT_DRAWABLE_HPP_
#define _TEST_POINT_DRAWABLE_HPP_

#include "TableTransform.hpp"

class PointDrawable : public sf::Drawable {
  public:
    PointDrawable(const TableTransform &transform, Point2D<Meter> position, sf::Color color = sf::Color::Blue, int drawRadius = 10)
        : PointDrawable(transform.toScreenCoordinates(position), color, drawRadius) {}
    PointDrawable(sf::Vector2f screenPosition, sf::Color color = sf::Color::Blue, int drawRadius = 10) : m_circle(drawRadius) {
        m_circle.setFillColor(color);
        m_circle.setOrigin(drawRadius, drawRadius);
        setPosition(screenPosition);
    }

    void setPosition(const TableTransform &transform, Point2D<Meter> position) { setPosition(transform.toScreenCoordinates(position)); }
    void setPosition(sf::Vector2f screenPosition) { m_circle.setPosition(screenPosition); }

  protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override { target.draw(m_circle, states); }

  private:
    sf::CircleShape m_circle;
};

#endif