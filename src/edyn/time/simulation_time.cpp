#include "edyn/util/island_util.hpp"
#include "edyn/simulation/island_coordinator.hpp"

namespace edyn {

double get_simulation_timestamp(entt::registry &registry) {
    auto &coordinator = registry.ctx().at<island_coordinator>();
    auto worker_time = coordinator.get_simulation_timestamp();
    return worker_time;
}

}
