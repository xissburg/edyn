#ifndef EDYN_DYNAMICS_ISLAND_SOLVER_HPP
#define EDYN_DYNAMICS_ISLAND_SOLVER_HPP

#include "edyn/math/scalar.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

class atomic_counter_sync;

void run_island_solver_seq_mt(entt::registry &, entt::entity island_entity,
                              unsigned num_iterations, unsigned num_position_iterations,
                              scalar dt, atomic_counter_sync *counter);

void run_island_solver_seq(entt::registry &, entt::entity island_entity,
                           unsigned num_iterations, unsigned num_position_iterations,
                           scalar dt);

}

#endif // EDYN_DYNAMICS_ISLAND_SOLVER_HPP
