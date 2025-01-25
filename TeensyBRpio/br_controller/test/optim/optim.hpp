#include <iostream>

/**
 * Optimizes the cost of `start` using a pseudo-local search.
 * 
 * This is experimental and subject to change.
 * 
 * @param start The initial guess
 * @param step_hint The greater, the more spaced the successive guesses will be.
 * 
 * @param T Must expose methods `T randomNeighbour(double dist_hint)` and `double cost(optional<double> limit = nullopt)`
 */
template <typename T>
T optimize_local_search(T start, double step_hint, unsigned int num_steps) {
    double_t cost = start.cost();
    for (unsigned int i = 0; i < num_steps; i++) {
        T neighbour = start.randomNeighbour(step_hint);
        double_t new_cost = neighbour.cost(cost);
        if (new_cost < cost) {
            start = neighbour;
            cost = new_cost;
        }
    }
    return start;
}