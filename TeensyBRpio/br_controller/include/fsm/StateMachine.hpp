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
 * The machine is created in an undefined state. It is undefined behaviour to call getCurrentState() before setCurrentState().
 * @tparam TState The base type of the machine's states. This can be a virtual type.
 * @tparam StateTypes The list of the possible states. Those must be concrete types.
 */
template <typename... StateTypes>
class StateMachine {
  private:
    using StateVariant = std::variant<StateTypes...>;
    StateVariant m_currentState;

  public:
    StateMachine() : m_currentState() {}

    template <typename TNewState, typename... Args>
    void setCurrentState(Args &&...args) {
        m_currentState.template emplace<TNewState>(std::forward<Args>(args)...);
    }

    template <class R, class Visitor>
    R visitCurrentState(Visitor &&visitor) const {
        return std::visit<R, Visitor>(std::forward<Visitor>(visitor), m_currentState);
    }
    template <class R, class Visitor>
    R visitCurrentState(Visitor &&visitor) {
        return std::visit<R, Visitor>(std::forward<Visitor>(visitor), m_currentState);
    }

    template <typename R>
    R getStateStatus() const {
        return visitCurrentState<R>(                                                //
            overload{[](const StateUninit &state) { return state.getStatus<R>(); }, //
                     [](const auto &state) { return state.getStatus(); }});
    } // namespace fsm

    template <typename R, typename... Args>
    R updateState(Args &&...args) {
        return visitCurrentState<R>( //
            overload{[](StateUninit &state) { return state.update<R>(); },
                     [&args...](auto &state) { return state.update(std::forward<Args>(args)...); }});
    }

    template <typename State>
    bool holdsState() const {
        return std::holds_alternative<State>(m_currentState);
    }

    template <typename State>
    const State *getStateOrNull() const {
        return std::get_if<State>(&m_currentState);
    }
    template <typename State>
    State *getStateOrNull() {
        return std::get_if<State>(&m_currentState);
    }
};

} // namespace fsm

#endif