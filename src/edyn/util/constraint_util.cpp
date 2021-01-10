#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/dirty.hpp"

namespace edyn {

namespace internal {
    void pre_make_constraint(entt::entity entity, entt::registry &registry, 
                            entt::entity body0, entt::entity body1, 
                            entt::entity *parent_entity) {

        registry.emplace<procedural_tag>(entity);

        // If the constraint has a parent (e.g. when it's a `contact_constraint` in a
        // contact manifold), only assign an `island_node_child` to it or else assign
        // an `island_node` since in that case it will be the root of the node sub-tree
        // into which `constraint_row`s will be added.
        if (parent_entity) {
            registry.emplace<island_node_child>(entity, *parent_entity);
            
            auto &node_parent = registry.get<island_node_parent>(*parent_entity);
            node_parent.children.insert(entity);
        } else {
            registry.emplace<island_node>(entity, entity_set{body0, body1});
        }

        // A constraint is a parent of its rows, thus it needs an `island_node_parent`.
        registry.emplace<island_node_parent>(entity);
        registry.emplace<island_container>(entity);

        if (!parent_entity) {
            // When there's no parent entity, the constraint is associated with the
            // rigid bodies directly.
            auto &node0 = registry.get<island_node>(body0);
            node0.entities.insert(entity);

            auto &node1 = registry.get<island_node>(body1);
            node1.entities.insert(entity);
            
            registry.get_or_emplace<dirty>(body0).updated<island_node>();
            registry.get_or_emplace<dirty>(body1).updated<island_node>();
            limit_dirty_to_island_of_procedural(registry, body0, body1);
        }

        auto &constraint_dirty = registry.get_or_emplace<dirty>(entity)
            .set_new()
            .created<procedural_tag, constraint, island_node_parent, island_container>();

        if (parent_entity) {
            constraint_dirty.created<island_node_child>();
            registry.get_or_emplace<dirty>(*parent_entity).updated<island_node_parent>();
        } else {
            constraint_dirty.created<island_node>();
        }
    }
}

void limit_dirty_to_island_of_procedural(entt::registry &registry, entt::entity ent0, entt::entity ent1) {
    if (!registry.has<procedural_tag>(ent0)) {
        EDYN_ASSERT(registry.has<procedural_tag>(ent1));
        auto &container = registry.get<island_container>(ent1);
        if (!container.entities.empty()) {
            registry.get_or_emplace<dirty>(ent0).islands(*container.entities.begin());
        }
    }

    if (!registry.has<procedural_tag>(ent1)) {
        EDYN_ASSERT(registry.has<procedural_tag>(ent0));
        auto &container = registry.get<island_container>(ent0);
        if (!container.entities.empty()) {
            registry.get_or_emplace<dirty>(ent1).islands(*container.entities.begin());
        }
    }
}

entt::entity add_constraint_row(entt::entity entity, constraint &con, entt::registry &registry, int priority) {
    EDYN_ASSERT(con.num_rows() + 1 < max_constraint_rows);
    EDYN_ASSERT(con.row[con.num_rows()] == entt::null);

    auto row_entity = registry.create();
    con.row[con.num_rows()] = row_entity;

    auto &row = registry.emplace<constraint_row>(row_entity);
    row.entity = con.body;
    row.priority = priority;

    registry.emplace<procedural_tag>(row_entity);

    // The constraint row is a child of the constraint.
    auto &row_child_node = registry.emplace<island_node_child>(row_entity);
    row_child_node.parent = entity;

    auto &con_parent_node = registry.get<island_node_parent>(entity);
    con_parent_node.children.insert(row_entity);

    registry.emplace<island_container>(row_entity);

    registry.get_or_emplace<dirty>(entity).updated<island_node_parent>();

    registry.get_or_emplace<dirty>(row_entity)
        .set_new()
        .created<island_node_child, island_container, procedural_tag, constraint_row>();

    return row_entity;
}

entt::entity make_contact_manifold(entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    auto manifold_entity = registry.create();
    make_contact_manifold(manifold_entity, registry, body0, body1, separation_threshold);
    return manifold_entity;
}

void make_contact_manifold(entt::entity manifold_entity, entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    EDYN_ASSERT(registry.valid(body0) && registry.valid(body1));
    registry.emplace<procedural_tag>(manifold_entity);
    registry.emplace<island_node>(manifold_entity, entity_set{body0, body1});
    registry.emplace<island_node_parent>(manifold_entity);
    registry.emplace<island_container>(manifold_entity);
    registry.emplace<contact_manifold>(manifold_entity, body0, body1, separation_threshold);

    // Assign a reference to the contact entity in the body nodes.
    auto &node0 = registry.get<island_node>(body0);
    node0.entities.insert(manifold_entity);

    auto &node1 = registry.get<island_node>(body1);
    node1.entities.insert(manifold_entity);

    // Mark stuff as dirty to schedule an update in the island worker.
    registry.get_or_emplace<dirty>(body0).updated<island_node>();
    registry.get_or_emplace<dirty>(body1).updated<island_node>();

    registry.get_or_emplace<dirty>(manifold_entity)
        .set_new()
        .created<procedural_tag, island_node, island_node_parent, island_container, contact_manifold>();

    limit_dirty_to_island_of_procedural(registry, body0, body1);
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