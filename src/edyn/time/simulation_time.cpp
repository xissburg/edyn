#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/simulation/island_coordinator.hpp"

namespace edyn {

double get_simulation_timestamp(entt::registry &registry) {
    if (registry.ctx().contains<island_coordinator>()) {
        auto &coordinator = registry.ctx().at<island_coordinator>();
        auto worker_time = coordinator.get_simulation_timestamp();
        return worker_time;
    } else if (registry.ctx().contains<stepper_sequential>()) {
        auto &stepper = registry.ctx().at<stepper_sequential>();
        return stepper.get_timestamp();
    }
    EDYN_ASSERT(false);
}

}
