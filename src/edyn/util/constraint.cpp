#include "edyn/util/constraint.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

entt::entity add_constraint_row(entt::entity entity, constraint &con, entt::registry &registry, int priority) {
    EDYN_ASSERT(con.num_rows() + 1 < max_constraint_rows);
    EDYN_ASSERT(con.row[con.num_rows()] == entt::null);

    auto row_entity = registry.create();
    con.row[con.num_rows()] = row_entity;

    auto &row = registry.emplace<constraint_row>(row_entity);
    row.entity = con.body;
    row.priority = priority;

    // Add island node and make it reference the constraint and vice-versa.
    registry.emplace<procedural_tag>(row_entity);
    auto &row_node = registry.emplace<island_node>(row_entity);
    row_node.entities.push_back(entity);

    auto &con_node = registry.get<island_node>(entity);
    con_node.entities.push_back(row_entity);

    registry.get_or_emplace<island_node_dirty>(entity).indexes.insert(entt::type_index<island_node>::value());
    
    auto &row_dirty = registry.get_or_emplace<island_node_dirty>(row_entity);
    row_dirty.is_new_entity = true;
    row_dirty.indexes.insert(entt::type_index<island_node>::value());
    row_dirty.indexes.insert(entt::type_index<procedural_tag>::value());

    return row_entity;
}

entt::entity make_contact_manifold(entt::registry &registry, entt::entity e0, entt::entity e1, scalar separation_threshold) {
    auto manifold_entity = registry.create();
    make_contact_manifold(manifold_entity, registry, e0, e1, separation_threshold);
    return manifold_entity;
}

void make_contact_manifold(entt::entity manifold_entity, entt::registry &registry, entt::entity e0, entt::entity e1, scalar separation_threshold) {
    EDYN_ASSERT(registry.valid(e0) && registry.valid(e1));
    registry.emplace<procedural_tag>(manifold_entity);
    registry.emplace<island_node>(manifold_entity, std::vector<entt::entity>{e0, e1});
    registry.emplace<contact_manifold>(manifold_entity, e0, e1, separation_threshold);

    // Assign a reference to the contact entity in the body nodes.
    auto &node0 = registry.get<island_node>(e0);
    node0.entities.push_back(manifold_entity);

    auto &node1 = registry.get<island_node>(e1);
    node1.entities.push_back(manifold_entity);

    // Mark stuff as dirty to schedule an update in the island worker.
    registry.get_or_emplace<island_node_dirty>(e0).indexes.insert(entt::type_index<island_node>::value());
    registry.get_or_emplace<island_node_dirty>(e1).indexes.insert(entt::type_index<island_node>::value());

    auto &contact_dirty = registry.get_or_emplace<island_node_dirty>(manifold_entity);
    contact_dirty.is_new_entity = true;
    contact_dirty.indexes.insert(entt::type_index<island_node>::value());
    contact_dirty.indexes.insert(entt::type_index<contact_manifold>::value());
    contact_dirty.indexes.insert(entt::type_index<procedural_tag>::value());
}

void set_constraint_enabled(entt::entity entity, entt::registry &registry, bool enabled) {
    auto& con = registry.get<constraint>(entity);
    
    if (enabled) {
        registry.remove<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows(); ++i) {
            registry.remove<disabled_tag>(con.row[i]);
        }
    } else {
        registry.emplace_or_replace<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows(); ++i) {
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