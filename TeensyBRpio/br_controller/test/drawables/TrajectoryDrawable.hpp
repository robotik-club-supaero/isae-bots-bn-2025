#ifndef _TEST_TRAJECTORY_DRAWABLE_HPP_
#define _TEST_TRAJECTORY_DRAWABLE_HPP_

#include "TableTransform.hpp"
#include "trajectories/Trajectory.hpp"

class TrajectoryDrawable : public sf::Drawable {
  public:
    TrajectoryDrawable(const TableTransform &transform, Trajectory &trajectory, double_t step, sf::Color color = DEFAULT_COLOR)
        : TrajectoryDrawable(color) {
        appendTrajectory(transform, trajectory, step);
    }

  protected:
    static const sf::Color DEFAULT_COLOR;
    TrajectoryDrawable(sf::Color color = DEFAULT_COLOR) : m_vertices(sf::LinesStrip), m_color(color) {}
    void appendTrajectory(const TableTransform &transform, Trajectory &trajectory, double_t step) {
        while (trajectory.advance(step)) {
            m_vertices.append(sf::Vertex(transform.toScreenCoordinates(trajectory.getCurrentPosition()), m_color));
        }
    }

    void draw(sf::RenderTarget &target, sf::RenderStates states) const override { target.draw(m_vertices, states); }

  private:
    sf::VertexArray m_vertices;
    sf::Color m_color;
};

const sf::Color TrajectoryDrawable::DEFAULT_COLOR(sf::Color::Red);

#endif