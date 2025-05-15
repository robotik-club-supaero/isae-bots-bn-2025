#ifndef _POSITION_ESTIMATOR_ODO_HPP_
#define _POSITION_ESTIMATOR_ODO_HPP_

#include "configuration.hpp"

#include "encoders/OdoEncoder.hpp"
#include "geometry/Position2D.hpp"
#include "math/LowPassFilter.hpp"

class MethodMoveFirst;
class MethodUpdateThetaFirst;
class LegacyMethod3;

/**
 * Estimates the position of a two-wheel robot based on two odometers. This class satisfies concept PositionFeedback.
 *
 * @tparam TMethod Either MethodMoveFirst, MethodUpdateThetaFirst or LegacyMethod3.
 */
template <OdoEncoder T, typename TMethod>
class PositionEstimatorOdo {
  public:
    PositionEstimatorOdo(T encoder, number_t ticksPerRad, number_t ticksPerMillimeter, number_t correctionFactorLR);

    PositionEstimatorOdo()
        requires std::is_default_constructible_v<T>
        : PositionEstimatorOdo(T(), ECARTS_ODOS, UNITS_ODOS, L_R_ODOS) {}

    /**
     * Read the counters from the encoder and update the estimated position.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(number_t interval);
    void resetPosition(Position2D<Meter> pos);

    Position2D<Meter> getRobotPosition() const;

    int32_t getLeftOdoCount() const;
    int32_t getRightOdoCount() const;

  private:
    friend class MethodMoveFirst;
    friend class MethodUpdateThetaFirst;
    friend class LegacyMethod3;

    number_t m_ticksPerRad;
    number_t m_ticksPerMillimeter;
    number_t m_correctionFactorLR;

    T m_encoder;
    LowPassFilter<number_t> m_filter;

    Position2D<Meter> m_position;

    int32_t m_odoLeftCount;
    int32_t m_odoRightCount;

    number_t m_positionThetaOdo;
    number_t m_positionThetaOffset;
};

/// Update x and y based on the current orientation, then update theta
class MethodMoveFirst {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, MethodMoveFirst> &estimator, number_t &dx, number_t &dy, number_t deltaL, number_t deltaR);
};

/// Update theta then infer the linear displacement
class MethodUpdateThetaFirst {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, MethodUpdateThetaFirst> &estimator, number_t &dx, number_t &dy, number_t deltaL,
                            number_t deltaR);
};

/// Ask Etienne Arlaud
class [[deprecated("Copied from old code base; not documented")]] LegacyMethod3 {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, LegacyMethod3> &estimator, number_t &dx, number_t &dy, number_t deltaL, number_t deltaR);
};

#endif