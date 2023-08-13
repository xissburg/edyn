#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/constraints/constraint_row_triple.hpp"
#include "edyn/constraints/constraint_row_with_spin.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/material_mixing.hpp"

namespace edyn {

namespace internal {
    bool pre_make_constraint(entt::registry &registry, entt::entity entity,
                             entt::entity body0, entt::entity body1) {
        // Multiple constraints of different types can be assigned to the same
        // entity. If this entity already has a graph edge, just do a few
        // consistency checks.
        if (registry.any_of<graph_edge>(entity)) {
        #if defined(EDYN_DEBUG) && !defined(EDYN_DISABLE_ASSERT)
            auto &edge = registry.get<graph_edge>(entity);
            auto [ent0, ent1] = registry.ctx().at<entity_graph>().edge_node_entities(edge.edge_index);
            EDYN_ASSERT(ent0 == body0 && ent1 == body1);
        #endif
            return false;
        }

        // Assign graph edge.
        auto node_index0 = registry.get<graph_node>(body0).node_index;
        auto node_index1 = registry.get<graph_node>(body1).node_index;
        auto edge_index = registry.ctx().at<entity_graph>().insert_edge(entity, node_index0, node_index1);
        registry.emplace<graph_edge>(entity, edge_index);
        registry.emplace<island_resident>(entity);
        registry.emplace<constraint_tag>(entity);

        return true;
    }
}

entt::entity make_contact_manifold(entt::registry &registry,
                                   entt::entity body0, entt::entity body1,
                                   scalar separation_threshold) {
    auto manifold_entity = registry.create();
    make_contact_manifold(manifold_entity, registry, body0, body1, separation_threshold);
    return manifold_entity;
}

void make_contact_manifold(entt::entity manifold_entity, entt::registry &registry,
                           entt::entity body0, entt::entity body1,
                           scalar separation_threshold) {
    EDYN_ASSERT(registry.valid(body0) && registry.valid(body1));
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(body0) && registry.all_of<rigidbody_tag>(body1));
    // One of the bodies must be dynamic.
    EDYN_ASSERT(registry.all_of<dynamic_tag>(body0) || registry.all_of<dynamic_tag>(body1));

    // Tire must be in the first entry. This is an assumption made by the `contact_patch_constraint`.
    if (registry.all_of<tire_material>(body1)) {
        std::swap(body0, body1);
    }

    registry.emplace<contact_manifold>(manifold_entity, body0, body1, separation_threshold);
    registry.emplace<contact_manifold_events>(manifold_entity);

    auto material_view = registry.view<material>();

    // Only create contact constraint if bodies have material.
    if (!material_view.contains(body0) || !material_view.contains(body1)) {
        // If not, emplace a null constraint to ensure an edge will exist in
        // the entity graph.
        make_constraint<null_constraint>(registry, manifold_entity, body0, body1);
        return;
    }

    auto &material0 = material_view.get<material>(body0);
    auto &material1 = material_view.get<material>(body1);

    auto &material_table = registry.ctx().at<material_mix_table>();
    auto restitution = scalar(0);

    if (auto *material = material_table.try_get({material0.id, material1.id})) {
        restitution = material->restitution;
    } else {
        restitution = material_mix_restitution(material0.restitution, material1.restitution);
    }

    if (restitution > EDYN_EPSILON) {
        registry.emplace<contact_manifold_with_restitution>(manifold_entity);
    }

    // Assign contact constraint to manifold.
    if (registry.any_of<tire_material>(body0) || registry.any_of<tire_material>(body1)) {
        make_constraint<contact_patch_constraint>(registry, manifold_entity, body0, body1);
    } else {
        make_constraint<contact_constraint>(registry, manifold_entity, body0, body1);
    }
}

void swap_manifold(contact_manifold &manifold) {
    std::swap(manifold.body[0], manifold.body[1]);

    manifold.each_point([](contact_point &cp) {
        std::swap(cp.pivotA, cp.pivotB);
        std::swap(cp.featureA, cp.featureB);
        cp.normal *= -1; // Point towards new A.

        if (cp.normal_attachment == contact_normal_attachment::normal_on_A) {
            cp.normal_attachment = contact_normal_attachment::normal_on_B;
        } else if (cp.normal_attachment == contact_normal_attachment::normal_on_B) {
            cp.normal_attachment = contact_normal_attachment::normal_on_A;
        }
    });
}

scalar get_effective_mass(const constraint_row &row) {
    return get_effective_mass(row.J, row.inv_mA, row.inv_IA, row.inv_mB, row.inv_IB);
}

scalar get_effective_mass(const std::array<vector3, 4> &J,
                          scalar inv_mA, const matrix3x3 &inv_IA,
                          scalar inv_mB, const matrix3x3 &inv_IB) {
    auto J_invM_JT = dot(J[0], J[0]) * inv_mA +
                     dot(inv_IA * J[1], J[1]) +
                     dot(J[2], J[2]) * inv_mB +
                     dot(inv_IB * J[3], J[3]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

scalar get_effective_mass(const std::array<vector3, 6> &J,
                          scalar inv_mA, const matrix3x3 &inv_IA,
                          scalar inv_mB, const matrix3x3 &inv_IB,
                          scalar inv_mC, const matrix3x3 &inv_IC) {
    auto J_invM_JT = dot(J[0], J[0]) * inv_mA +
                     dot(inv_IA * J[1], J[1]) +
                     dot(J[2], J[2]) * inv_mB +
                     dot(inv_IB * J[3], J[3]) +
                     dot(J[4], J[4]) * inv_mC +
                     dot(inv_IC * J[5], J[5]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

scalar get_relative_speed(const std::array<vector3, 4> &J,
                          const vector3 &linvelA,
                          const vector3 &angvelA,
                          const vector3 &linvelB,
                          const vector3 &angvelB) {
    auto relspd = dot(J[0], linvelA) +
                  dot(J[1], angvelA) +
                  dot(J[2], linvelB) +
                  dot(J[3], angvelB);
    return relspd;
}

}
