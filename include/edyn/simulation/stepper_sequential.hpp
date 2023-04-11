#ifndef EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP
#define EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/dynamics/solver.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"

namespace edyn {

/**
 * Steps the simulation sequentially in the main thread into the given
 * registry. It can optionally parallelize many steps of the simulation.
 */
class stepper_sequential {
public:
    stepper_sequential(entt::registry &registry, double time, bool multithreaded);

    void update(double time);
    void step_simulation(double time);
    void set_paused(bool paused);

    bool is_paused() const {
        return m_paused;
    }

    double get_simulation_timestamp() const {
        return m_last_time - m_accumulated_time;
    }

    auto & get_island_manager() {
        return m_island_manager;
    }

private:
    entt::registry *m_registry;
    island_manager m_island_manager;
    polyhedron_shape_initializer m_poly_initializer;
    solver m_solver;

    double m_accumulated_time {};
    double m_last_time {};
    bool m_multithreaded;
    bool m_paused;
};

}

#endif // EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP
