#ifndef _MULTI_CURVE_TRAJECTORY_HPP_
#define _MULTI_CURVE_TRAJECTORY_HPP_

#include "stl/SmallDeque.hpp"
#include "trajectories/CurveTrajectory.hpp"

#include <concepts>

/**
 * Possibly lazy contiguous curve generator.
 * It is the responsibility of the implementation to ensure the curves are contiguous.
 *
 * It is undefined behavior to call `next()` unless `hasNext()` returns true.
 * Calling `next()` invalidates the value of `hasNext()`.
 *
 * @tparam TCurve Must be a complete type.
 */
template <typename T, typename TCurve>
concept CurveGenerator = requires(T &t, const T &t_const) {
    { t.next() } -> std::convertible_to<TCurve>;
    { t_const.hasNext() } -> std::convertible_to<bool>;
};

#ifndef ARDUINO

#include <vector>

/**
 * A wrapper around a vector of pre-generated curves that implements concept CurveGenerator.
 */
template <Curve TCurve>
class GeneratorVector {
  public:
    GeneratorVector(std::vector<TCurve> curves) : m_curves(std::move(curves)), m_index() {}

    template <class InputIt>
    GeneratorVector(InputIt first, InputIt last) : GeneratorVector(std::vector<TCurve>(first, last)) {}

    GeneratorVector(std::initializer_list<TCurve> curves) : GeneratorVector(std::vector<TCurve>(curves)) {}

    TCurve next() {
        TCurve curve = std::move(m_curves[m_index++]);
        return curve;
    }
    bool hasNext() const { return m_index < m_curves.size(); }

  private:
    std::vector<TCurve> m_curves;
    std::size_t m_index;
};

#endif

/**
 * A trajectory that follows contiguous, possibly lazily generated, differentiable parametric curves.
 *
 * It is the responsibility of the generator to ensure the resulting trajectory is smooth enough.
 * At the bare minimum, the position on the trajectory (including the local direction) must be a continuous function.
 *
 * @tparam TCurve,TGenerator Must be complete types
 * See concept CurveGenerator.
 */
#ifndef ARDUINO
template <Curve TCurve, CurveGenerator<TCurve> TGenerator = GeneratorVector<TCurve>>
#else
template <Curve TCurve, CurveGenerator<TCurve> TGenerator>
#endif
class MultiCurveTrajectory : public SmoothTrajectory {
  public:
    using size_type = std::size_t;

    /**
     * @param generator Must generate at least one curve (i.e. `generator.hasNext()` must be true when given to this constructor).
     */
    MultiCurveTrajectory(TGenerator generator)
        : m_currentCurve(generator.next()), m_generator(std::move(generator)), m_generatedCurves(), m_pendingCurvesLength(-1) {}

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override final {
        m_currentCurve.forceGenerate(distance);
        while (std::optional<double_t> remainingOnCurrentCurve = m_currentCurve.getRemainingDistance()) {
            if (distance < *remainingOnCurrentCurve || !setupNextCurve()) {
                break;
            }
            distance -= *remainingOnCurrentCurve;
            m_currentCurve.forceGenerate(distance);
        }
        if (m_pendingCurvesLength < 0) {
            computeRemainingDistance();
        }

        return m_currentCurve.advance(distance) || setupNextCurve();
    }

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override final { return m_currentCurve.getCurrentPosition(); }

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override final {
        if (m_pendingCurvesLength < 0 || m_generator.hasNext()) {
            return std::nullopt;
        };
        if (auto remainingOnCurrentCurve = m_currentCurve.getRemainingDistance()) {
            return *remainingOnCurrentCurve + m_pendingCurvesLength;
        } else {
            return std::nullopt;
        }
    }

    /// @copydoc Trajectory::getMaxCurvature()
    /// This generates the next `distance` meters of the trajectory if they are not already generated.
    double_t getMaxCurvature(double_t distance) override final {
        double_t curvature = 0;
        size_type i = 0;
        while (distance >= 0) {
            std::optional<std::reference_wrapper<trajectory_t>> curve_opt = getCurve(i);
            if (!curve_opt) {
                break;
            }
            trajectory_t &curve = curve_opt->get();

            curvature = std::max(curvature, curve.getMaxCurvature(distance));

            if (auto remainingOnCurve = curve.getRemainingDistance()) {
                distance -= *remainingOnCurve;
            } else {
                // getMaxCurvature generates the trajectory, so this means the trajectory is strictly longer than `distance`.
                break;
            }
            ++i;
        }

        return curvature;
    }

    const TCurve &getCurrentCurve() const { return m_currentCurve.getCurve(); }

    /**
     * Returns the curve at position `index`. The `0-th` curve is the current curve.
     * This does not generate the curve if it is not already generated.
     *
     * This is undefined behavior if `index >= numOfGeneratedCurves()`.
     */
    const TCurve &getGeneratedCurve(size_type index) const { return getGeneratedTrajectory(index).getCurve(); }

    /**
     * Returns the number of generated oncoming curves, including the current curve.
     *
     * This only includes the curves generated so far and there may be more curves pending to be generated.
     * Call `forceGenerate` first if you need to know the total number of curves.
     */
    size_type numOfGeneratedCurves() const { return m_generatedCurves.size() + 1; }

    /**
     * Advances the trajectory to the next curve.
     *
     * @return If the current curve is already the last one, this does nothing and returns false. Otherwise, this returns true.
     */
    bool skipToNextCurve() { return setupNextCurve(); }

    /**
     * Immediately generates all the curves. This does not change the current position.
     *
     * This is not cheap, because all the pending curves will be sampled immediately.
     * This won't return if the trajectory has an infinite number of curves.
     */
    void forceGenerate() {
        while (m_generator.hasNext()) {
            m_generatedCurves.push_back(m_generator.next());
        }
        for (auto it = m_generatedCurves.begin(); it != m_generatedCurves.end(); ++it) {
            it->forceGenerate();
        }
        m_currentCurve.forceGenerate();
    }

  protected:
    const TGenerator &getGenerator() const { return m_generator; }
    TGenerator &getGenerator() { return m_generator; }

  private:
    using trajectory_t = CurveTrajectory<TCurve>;

    bool setupNextCurve() {
        if (m_generatedCurves.empty()) {
            if (!m_generator.hasNext()) {
                return false;
            }
            m_currentCurve = m_generator.next();
        } else {
            m_currentCurve = std::move(m_generatedCurves.front());
            m_generatedCurves.pop_front();
        }

        computeRemainingDistance();
        return true;
    }
    void computeRemainingDistance() {
        if (!m_generator.hasNext()) {
            m_pendingCurvesLength = 0;
            for (const auto &curve : m_generatedCurves) {
                if (auto distance = curve.getRemainingDistance()) {
                    m_pendingCurvesLength += *distance;
                } else {
                    m_pendingCurvesLength = -1;
                    break;
                }
            }
        }
    }

    const CurveTrajectory<TCurve> &getGeneratedTrajectory(size_type index) const {
        if (index == 0) {
            return m_currentCurve;
        } else {
            return m_generatedCurves[index - 1];
        }
    }

    /**
     * Returns the curve at position `index`. The `0-th` curve is the current curve.
     * This generates the next `index - 1` curves if they are not generated yet.
     *
     * This returns an empty optional if there are less than `index` remaining curves after the current curve.
     */
    std::optional<std::reference_wrapper<trajectory_t>> getCurve(size_type index) {
        if (index == 0) {
            return m_currentCurve;
        } else {
            while (index > m_generatedCurves.size()) {
                if (!m_generator.hasNext()) {
                    return std::nullopt;
                }
                if (!m_generatedCurves.reserve(1)) {
                    // Not enough memory to generate the curve. (This should not happen)
                    // Eventually, when the current curve is complete and "popped", free memory will become sufficient again.
                    return std::nullopt;
                }
                // We made sure the capacity was sufficient above, so this can't fail.
                std::ignore = m_generatedCurves.push_back(m_generator.next());
            }
            return m_generatedCurves[index - 1];
        }
    }

    trajectory_t m_currentCurve;
    TGenerator m_generator;
    SmallDeque<trajectory_t> m_generatedCurves;
    double_t m_pendingCurvesLength;
};

#endif