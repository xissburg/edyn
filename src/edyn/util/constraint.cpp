#include "edyn/util/constraint.hpp"

namespace edyn {

entt::entity add_constraint_row(entt::entity entity, constraint &con, entt::registry &registry, int priority) {
    EDYN_ASSERT(con.num_rows + 1 < max_constraint_rows);

    auto row_entity = registry.create();
    con.row[con.num_rows++] = row_entity;
    
    auto &row = registry.emplace<constraint_row>(row_entity);
    row.entity = con.body;
    row.priority = priority;

    // Add island node and make it reference the constraint and vice-versa.
    auto &row_node = registry.emplace<island_node>(row_entity, true);
    row_node.entities.push_back(entity);

    auto &con_node = registry.get<island_node>(entity);
    con_node.entities.push_back(row_entity);

    registry.get_or_emplace<island_node_dirty>(entity).indexes.push_back(entt::type_index<island_node>::value());

    return entity;
}

void set_constraint_enabled(entt::entity entity, entt::registry &registry, bool enabled) {
    auto& con = registry.get<constraint>(entity);
    
    if (enabled) {
        registry.remove<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.remove<disabled_tag>(con.row[i]);
        }
    } else {
        registry.emplace_or_replace<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.emplace_or_replace<disabled_tag>(con.row[i]);
        }
    }
}

scalar get_effective_mass(const constraint_row &row, 
                          const mass_inv &inv_mA, const inertia_world_inv &inv_IA,
                          const mass_inv &inv_mB, const inertia_world_inv &inv_IB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * inv_mA +
                     dot(inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * inv_mB +
                     dot(inv_IB * row.J[3], row.J[3]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

}