#include "edyn/util/insert_material_mixing.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material) {
    auto &material_table = registry.ctx().at<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->material_table_changed();
    }
}

}
