#ifndef _CONTROLLER_MANAGER_HPP_
#define _CONTROLLER_MANAGER_HPP_

#include "configuration.hpp"
#include "defines/constraint.hpp"

#include "Actuators.hpp"
#include "Clock.hpp"
#include "PositionFeedback.hpp"
#include "fsm/StateMachine.hpp"

#include "manager/ManagerState.hpp"
#include "manager/states/StateActivating.hpp"
#include "manager/states/StateActive.hpp"
#include "manager/states/StateDeactivating.hpp"
#include "manager/states/StateIdle.hpp"

#include <optional>

namespace manager {

/**
 * A manager for a generic closed-loop controller. This class manages the state of the actuators, connects the actuators to the output
 * of the controller and provides the controller with the position feedback from the robot.
 *
 * The manager must be activated with setActive(true) before the controller can send commands to the actuators.
 *
 * This class is designed to be as generic as possible, hence the number of type parameters.
 *
 * @tparam TActuators See concept Actuators. Must be a concrete type.
 * @tparam TController The controller. See UnicycleController. Must be a concrete type.
 * @tparam TFeedback See concept PositionFeedback. Must be a concrete type.
 * @tparam TClock The clock to use to get time measurements. Must be a concrete type.
 *
 * Using virtual types as any of the type parameters is undefined behaviour.
 */
template <Actuators TActuators, CanControl<TActuators> TController, PositionFeedback TFeedback, Clock TClock>
class ControllerManager
    : private fsm::StateMachine<StateIdle<TActuators>, StateDeactivating<TActuators>, StateActivating<TActuators>, StateActive<TActuators>> {
  public:
    using controller_t = TController;
    using actuators_t = TActuators;
    using feedback_t = TFeedback;
    using _clock_t = TClock;

    /**
     * @param updateInterval The interval (in microseconds) between two updates of the command to send to the motors. Multiple calls to update()
     * within this interval will result in the command being updated only once. In other words, `interval` will always have the same value when
     * calling "feedback.update()", "controller.updateCommand()" and "actuators.update()".
     */
    ControllerManager(duration_t updateInterval, TClock clock, TController controller, TActuators actuators, TFeedback feedback);

    ControllerManager(duration_t minUpdateInterval, duration_t maxUpdateInterval, TClock clock, TController controller, TActuators actuators,
                      TFeedback feedback);

    /// Initializes the manager with the default values from the configuration file.
    ControllerManager()
        requires Default<TClock> && Default<TController> && Default<TFeedback> && std::is_constructible_v<TActuators, const TFeedback &>
        : ControllerManager(create()) {}

  private:
    /// Initializes the manager with the default values from the configuration file.
    ControllerManager(TActuators actuators, TFeedback feedback)
        requires Default<TClock> && Default<TController>
        : ControllerManager(UPDATE_INTERVAL, TClock(), TController(), std::move(actuators), std::move(feedback)) {}

    static ControllerManager create()
        requires Default<TClock> && Default<TController> && Default<TFeedback> && std::is_constructible_v<TActuators, const TFeedback &>
    {
        TFeedback feedback;
        TActuators actuators(feedback);
        return ControllerManager(std::move(actuators), std::move(feedback));
    }

  public:
    ManagerStatus getStatus() const;
    bool isActive() const { return getStatus() == Active; }
    void setActive(bool active);

    /**
     * Sends an order to the controller if this manager is active. If the manager is not active, this does nothing.
     *
     * @param order A callable that takes the controller and the current robot position as parameters.
     * @return true if the order was sent, false otherwise.
     */
    template <typename Fun>
        requires std::invocable<Fun &, TController &, Position2D<Meter>>
    bool sendOrder(Fun order) {
        if (isActive()) {
            order(m_controller, m_feedback.getRobotPosition());
            return true;
        } else {
            log(WARN, "The order cannot be processed due to the current state of the manager.");
            return false;
        }
    }

    /**
     * Updates the state of the manager based on the manager's clock and tick interval.
     * If the manager is active, this also updates and sends the command to the actuators.
     *
     * @return If the time elapsed since the last update is less than the minimum tick interval, this does nothing
     * and return false. Otherwise, this does exactly one udpate and returns true.
     *
     * If the time elapsed since the last update was greater than the maximum tick interval, this still does only one update,
     * and this function must be called again to catch up on overdue updates.
     */
    bool update();

    /**
     * Updates the state of the manager based on the provided interval.
     * If the manager is active, this also updates and sends the command to the actuators.
     *
     * This bypasses the manager's clock and tick interval and allows the use of an external clock (especially in tests).
     *
     * # Caveat
     *
     * This desynchronizes the manager from its internal clock. Calling `update(void)` after `update(double_t)` is a logic error
     * and will lead to inconsistent update rates. To avoid this, you must call `resyncClock()` before calling `update(void)` again.
     *
     * @param interval The time elapsed since the last update. Must be positive.
     */
    void update(double_t interval);

    /**
     * Resynchronizes the manager with its internal clock. This resets the time of the last update.

     * This should be called when `update(void)` has not been called for a while and you want to skip overdue ticks.
     */
    void resyncClock();

    void resetPosition(Position2D<Meter> newPosition);

    // Getters and setters

    duration_t getMinUpdateInterval() const;
    duration_t getMaxUpdateInterval() const;

    void setMinUpdateInterval(duration_t updateInterval);
    void setMaxUpdateInterval(duration_t updateInterval);
    /// Sets both the min and max update intervals (in µs)
    void setUpdateInterval(duration_t updateInterval);

    const TController &getController() const;
    /**
     * This function can be used to make persistent changes to the controller even when the manager is inactive.
     * It should not be used to to update the command or start a displacement. Use sendOrder instead.
     */
    TController &getController();
    const TActuators &getActuators() const;
    const TFeedback &getPositionFeedback() const;

    const TClock &getClock() const;

  private:
    using Self = ControllerManager<TActuators, TController, TFeedback, TClock>;
    using StateIdle = manager::StateIdle<TActuators>;
    using StateDeactivating = manager::StateDeactivating<TActuators>;
    using StateActivating = manager::StateActivating<TActuators>;
    using StateActive = manager::StateActive<TActuators>;

    TClock m_clock;
    duration_t m_minUpdateInterval; // µS
    duration_t m_maxUpdateInterval; // µS
    std::optional<instant_t> m_lastUpdate;

    TController m_controller;
    TActuators m_actuators;
    TFeedback m_feedback;
};

} // namespace manager

#endif