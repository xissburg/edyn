#ifndef EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP
#define EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/collision/raycast_service.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"

namespace edyn {

class stepper_sequential {

    void step_simulation(double dt);

public:
    stepper_sequential(entt::registry &registry);

    void update();

private:
    entt::registry *m_registry;
    island_manager m_island_manager;
    raycast_service m_raycast_service;
    polyhedron_shape_initializer m_poly_initializer;
    solver m_solver;

    double m_accumulated_time {};
    double m_last_time {};
    bool m_multithreaded;
};

}

#endif // EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP
