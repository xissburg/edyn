#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/simulation/stepper_async.hpp"

namespace edyn {

double get_simulation_timestamp(entt::registry &registry) {
    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        return stepper->get_simulation_timestamp();
    }

    if (auto *stepper = registry.ctx().find<stepper_sequential>()) {
        return stepper->get_simulation_timestamp();
    }

    return 0;
}

}
