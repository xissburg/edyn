#include "edyn/util/island_util.hpp"
#include "edyn/parallel/island_coordinator.hpp"

namespace edyn {

double get_island_worker_timestamp(entt::registry &registry) {
    auto &coordinator = registry.ctx().at<island_coordinator>();
    auto worker_time = coordinator.get_worker_timestamp();
    return worker_time;
}

}
