#ifndef _FSM_STATE_MACHINE_HPP_
#define _FSM_STATE_MACHINE_HPP_

#include "defines/func.hpp"
#include "logging.hpp"

#include <variant>

namespace fsm {

struct StateUninit {
    template <typename R>
    R getStatus() const {
        abort("Cannot get status of StateUninit.");
    }
    template <typename R>
    R update() const {
        abort("Cannot update StateUninit.");
    }
};

/**
 * A state machine.
 *
 * @tparam StateTypes The list of the possible states. Those must be concrete types. The list may not be empty and the first type must be
 * default-constructible and is used as the initial state. You can use `StateUninit` as a fallback if you need to late-initialize the initial state
 * with some parameters.
 */
template <typename... StateTypes>
class StateMachine {
  private:
    using StateVariant = std::variant<StateTypes...>;
    StateVariant m_currentState;

  public:
    StateMachine() : m_currentState() {}

    /// Sets the current state
    template <typename TNewState, typename... Args>
    void setCurrentState(Args &&...args) {
        m_currentState.template emplace<TNewState>(std::forward<Args>(args)...);
    }

    /// Visits the current state. The behavior is similar to that of `std::visit` for variants.
    /// @{
    template <class R, class Visitor>
    R visitCurrentState(Visitor &&visitor) const {
        return std::visit<R, Visitor>(std::forward<Visitor>(visitor), m_currentState);
    }
    template <class R, class Visitor>
    R visitCurrentState(Visitor &&visitor) {
        return std::visit<R, Visitor>(std::forward<Visitor>(visitor), m_currentState);
    }
    /// @}

    /// Calls method `getStatus` on the current state and returns the result. All the possible state types must have such a method.
    /// If the current state is `StateUninit`, this does not return.
    template <typename R>
    R getStateStatus() const {
        return visitCurrentState<R>(                                                //
            overload{[](const StateUninit &state) { return state.getStatus<R>(); }, //
                     [](const auto &state) { return state.getStatus(); }});
    }

    /// Calls method `update` on the current state with the specified arguments and returns the result. All the possible state types must have such a
    /// method. If the current state is `StateUninit`, this does not return.
    template <typename R, typename... Args>
    R updateState(Args &&...args) {
        return visitCurrentState<R>( //
            overload{[](StateUninit &state) { return state.update<R>(); },
                     [&args...](auto &state) { return state.update(std::forward<Args>(args)...); }});
    }

    /// Indicates whether the current state has type `State`.
    template <typename State>
    bool holdsState() const {
        return std::holds_alternative<State>(m_currentState);
    }

    /// Returns a reference to the current state if it has type `State`, nullptr otherwise.
    /// @{
    template <typename State>
    const State *getStateOrNull() const {
        return std::get_if<State>(&m_currentState);
    }
    template <typename State>
    State *getStateOrNull() {
        return std::get_if<State>(&m_currentState);
    }
    /// @}
};

} // namespace fsm

#endif