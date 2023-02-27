#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/simulation/stepper_async.hpp"

namespace edyn {

double get_simulation_timestamp(entt::registry &registry) {
    if (registry.ctx().contains<stepper_async>()) {
        auto &stepper = registry.ctx().at<stepper_async>();
        auto worker_time = stepper.get_simulation_timestamp();
        return worker_time;
    } else if (registry.ctx().contains<stepper_sequential>()) {
        auto &stepper = registry.ctx().at<stepper_sequential>();
        return stepper.get_simulation_timestamp();
    }
    EDYN_ASSERT(false);
    return {};
}

}
