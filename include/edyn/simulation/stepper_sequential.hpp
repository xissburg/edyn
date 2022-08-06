#ifndef EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP
#define EDYN_SIMULATION_STEPPER_SEQUENTIAL_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/collision/raycast_service.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"

namespace edyn {

class stepper_sequential {
public:
    stepper_sequential(entt::registry &registry, bool multithreaded);
    void update();
    void step_simulation();
    void set_paused(bool paused);

    double get_timestamp() const {
        return m_last_time;
    }

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
