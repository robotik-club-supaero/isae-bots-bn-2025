#ifndef ARDUINO

#include "specializations/manager.hpp"
#include "trajectories/PathTrajectory.hpp"
#include "trajectories/PolygonalTrajectory.hpp"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

constexpr controller::ControllerStatus Still = controller::ControllerStatus::Still;

namespace PYBIND11_NAMESPACE {
namespace detail {
// Conversion SmallDeque<T> <-> Python list
template <typename T>
struct type_caster<SmallDeque<T>> : public list_caster<SmallDeque<T>, T> {};
} // namespace detail
} // namespace PYBIND11_NAMESPACE

PYBIND11_MODULE(br_trajectories, m) {
    m.doc() = "Python bindings to display and simulate trajectories. This does NOT allow to communicate with the program that runs the BR, you need "
              "to use ROS for that.";

    py::class_<Angle>(m, "Angle").def(py::init<double_t>()).def_property_readonly("value", &Angle::value);

    py::class_<Point2D<Meter>>(m, "Point2D")
        .def(py::init<>())
        .def(py::init<double_t, double_t>())
        .def("__repr__", &Point2D<Meter>::operator std::string)
        .def_readwrite("x", &Point2D<Meter>::x)
        .def_readwrite("y", &Point2D<Meter>::y)
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(py::self - py::self)
        .def(py::self -= py::self)
        .def(py::self * double_t())
        .def(double_t() * py::self)
        .def(py::self *= double_t())
        .def(py::self / double_t())
        .def(py::self /= double_t());

    py::class_<Position2D<Meter>, Point2D<Meter>>(m, "Position2D")
        .def(py::init<>())
        .def(py::init<double_t, double_t, double_t>())
        .def(py::init<Point2D<Meter>, double_t>())
        .def("__repr__", &Position2D<Meter>::operator std::string)
        .def_readwrite("theta", &Position2D<Meter>::theta);

    py::class_<BezierCurve<4>>(m, "BezierCurve")        //
        .def(py::init<std::array<Point2D<Meter>, 4>>()) //
        .def("at", &BezierCurve<4>::operator());

    py::class_<DisplacementKind>(m, "DisplacementKind")
        .def_property_readonly_static("FORWARD", []() { return FORWARD; })
        .def_property_readonly_static("REVERSE", []() { return REVERSE; });

    py::implicitly_convertible<double_t, Angle>();

    m.def("getTrajectoryCurves", [](Position2D<Meter> robotPosition, std::optional<Angle> finalOrientation, SmallDeque<Point2D<Meter>> path) {
        std::vector<BezierCurve<4>> curves;
        curves.reserve(path.size());

        path.push_front(robotPosition);

        // We use BezierCurvesGenerator instead of PathTrajectory, because we just want the Bézier curves and don't need
        // all the length and curvature sampling
        BezierCurvesGenerator trajectory(robotPosition.theta, finalOrientation, std::move(path));

        while (trajectory.hasNext()) {
            curves.push_back(trajectory.next());
        }
        return curves;
    });
    &m.def(
        "measureTrajectoryDuration",
        [](Position2D<Meter> robotPosition, SmallDeque<Point2D<Meter>> path, DisplacementKind kind, std::optional<Angle> finalOrientation,
           bool allowCurve, duration_t limitMicros) {
            path.push_front(robotPosition);

            manager_t manager = createManager(robotPosition);
            manager.sendOrder([&](controller_t &controller, Position2D<Meter> position) {
                controller.startPath(kind, allowCurve, position, finalOrientation, path);
            });

            duration_t time = 0;
            while (manager.getController().getStatus() != Still || manager.getController().isMoving()) {
                manager.update(UPDATE_INTERVAL / 1e6);
                time += UPDATE_INTERVAL;
                if (time > limitMicros) {
                    return std::numeric_limits<duration_t>::max();
                }
            }

            return time;
        },
        py::arg("robotPosition"), py::arg("path"), py::kw_only(), py::arg("kind") = FORWARD, py::arg("finalOrientation") = std::nullopt,
        py::arg("allowCurve") = true, py::arg("limitMicros") = 100000000);
}

void log(LogSeverity _severity, const char *_message) {
    // Ignore all logs
}

#endif