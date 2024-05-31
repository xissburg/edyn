#ifndef EDYN_TIME_SIMULATION_TIME_HPP
#define EDYN_TIME_SIMULATION_TIME_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Get timestamp of the last simulation step.
 * @param registry Data source.
 * @return Simulation timestamp.
 */
double get_simulation_timestamp(entt::registry &registry);

}

#endif // EDYN_TIME_SIMULATION_TIME_HPP
