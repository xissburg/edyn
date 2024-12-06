#include "edyn/util/insert_material_mixing.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material) {
    auto &material_table = registry.ctx().get<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->material_table_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_material_table(material_table);
    }
}

}
