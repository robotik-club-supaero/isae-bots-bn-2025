#ifndef _TEST_ROBOT_DRAWABLE_HPP_
#define _TEST_ROBOT_DRAWABLE_HPP_

#include "TableTransform.hpp"
#include "geometry/Position2D.hpp"

class RobotDrawable : public sf::Drawable {
  public:
    RobotDrawable(sf::Color color = sf::Color(128, 0, 128)) : m_robotMarker(sf::Vector2f(2 * DRAW_SIZE, 2 * DRAW_SIZE)), m_headMarker(sf::Vector2f(2 * HEAD_SIZE, DRAW_SIZE)) {
        m_robotMarker.setFillColor(color);
        m_robotMarker.setOrigin(DRAW_SIZE, DRAW_SIZE);
        m_headMarker.setFillColor(sf::Color::White);
    }
    RobotDrawable(const TableTransform &transform, Position2D<Meter> position) : RobotDrawable() { setPosition(transform, position); }

    void setPosition(const TableTransform &transform, Position2D<Meter> position) {
        setPosition(transform.toScreenCoordinates(position), position.theta);
    }
    void setPosition(sf::Vector2f screenPosition, Angle orientation) {
        number_t rotation = 180. / Angle::Pi * (Angle::Pi / 2 - orientation.value());

        m_robotMarker.setPosition(screenPosition);
        m_robotMarker.setRotation(rotation);

        m_headMarker.setPosition(screenPosition);
        m_headMarker.setRotation(rotation + 180);
    }

  protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override {
        target.draw(m_robotMarker, states);
        target.draw(m_headMarker, states);
    }

  private:
    sf::RectangleShape m_robotMarker;
    sf::RectangleShape m_headMarker;

    static constexpr unsigned int DRAW_SIZE = 20;
    static constexpr unsigned int HEAD_SIZE = 3;
};

#endif