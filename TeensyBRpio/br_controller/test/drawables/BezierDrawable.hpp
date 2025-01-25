#ifndef _TEST_BEZIER_DRAWABLE_HPP_
#define _TEST_BEZIER_DRAWABLE_HPP_

#include "TableTransform.hpp"
#include "geometry/Bezier.hpp"

/// Draws intermediary control points of a Bézier curve.
class BezierDrawable : public sf::Drawable {
  public:
    BezierDrawable(const TableTransform &transform, const BezierCurve &curve, sf::Color color = DEFAULT_COLOR) : m_controlPoints() {
        const std::vector<Point2D<Meter>> controlPoints = curve.points();
        for (std::size_t i = 1; i < controlPoints.size() - 1; i++) {
            sf::CircleShape sfPoint(4); // Radius of 4 pixels
            sfPoint.setFillColor(color);
            sfPoint.setPosition(transform.toScreenCoordinates(controlPoints[i]));
            sfPoint.move(-4, -4); // Center the circle
            m_controlPoints.push_back(sfPoint);
        }
    }

    static const sf::Color DEFAULT_COLOR;

  protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override {
        for (const auto &point : m_controlPoints) {
            target.draw(point, states);
        }
    }

  private:
    std::vector<sf::CircleShape> m_controlPoints;
};

const sf::Color BezierDrawable::DEFAULT_COLOR(0, 100, 0);

#endif