#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/constraints/constraint_row.hpp"

namespace edyn {

namespace internal {
    bool pre_make_constraint(entt::entity entity, entt::registry &registry,
                             entt::entity body0, entt::entity body1, bool is_graph_edge) {
        // Multiple constraints of different types can be assigned to the same
        // entity. If this entity already has a graph edge, just do a few
        // consistency checks.
        if (registry.has<graph_edge>(entity)) {
            auto &edge = registry.get<graph_edge>(entity);
            auto [ent0, ent1] = registry.ctx<entity_graph>().edge_node_entities(edge.edge_index);
            EDYN_ASSERT(ent0 == body0 && ent1 == body1);
            EDYN_ASSERT(registry.has<constraint_impulse>(entity));
            EDYN_ASSERT(registry.has<procedural_tag>(entity));
            return false;
        }

        registry.emplace<constraint_impulse>(entity);
        auto &con_dirty = registry.get_or_emplace<dirty>(entity);
        con_dirty.created<constraint_impulse>();

        // If the constraint is not a graph edge (e.g. when it's a `contact_constraint`
        // in a contact manifold), it means it is handled as a child of another entity
        // that is a graph edge and thus creating an edge for this would be redundant.
        if (is_graph_edge) {
            auto node_index0 = registry.get<graph_node>(body0).node_index;
            auto node_index1 = registry.get<graph_node>(body1).node_index;
            auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
            registry.emplace<procedural_tag>(entity);
            registry.emplace<graph_edge>(entity, edge_index);
            con_dirty.created<procedural_tag>();
        }

        return true;
    }
}

entt::entity make_contact_manifold(entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    auto manifold_entity = registry.create();
    make_contact_manifold(manifold_entity, registry, body0, body1, separation_threshold);
    return manifold_entity;
}

void make_contact_manifold(entt::entity manifold_entity, entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    EDYN_ASSERT(registry.has<rigidbody_tag>(body0) && registry.has<rigidbody_tag>(body1));
    // One of the bodies must be dynamic.
    EDYN_ASSERT(registry.has<dynamic_tag>(body0) || registry.has<dynamic_tag>(body1));

    // Tire must be in the first entry. This is an assumption made by the `contact_patch_constraint`.
    if (registry.has<tire_material>(body1)) {
        std::swap(body0, body1);
    }

    registry.emplace<procedural_tag>(manifold_entity);
    registry.emplace<contact_manifold>(manifold_entity, body0, body1, separation_threshold);

    auto node_index0 = registry.get<graph_node>(body0).node_index;
    auto node_index1 = registry.get<graph_node>(body1).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(manifold_entity, node_index0, node_index1);
    registry.emplace<graph_edge>(manifold_entity, edge_index);

    registry.get_or_emplace<dirty>(manifold_entity)
        .set_new()
        .created<procedural_tag,
                 contact_manifold>();
}

scalar get_effective_mass(const constraint_row &row) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

void prepare_row(constraint_row &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &linvelB,
                 const vector3 &angvelA, const vector3 &angvelB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB);

    auto restitution = restitution_curve(options.restitution, relvel);
    row.rhs = -(options.error * options.erp + relvel * (1 + restitution));
}

void apply_angular_impulse(scalar impulse,
                           constraint_row &row,
                           size_t ent_idx) {
    auto idx_J = ent_idx * 2 + 1;
    matrix3x3 inv_I;
    delta_angvel *dw;
    delta_spin *ds;

    if (ent_idx == 0) {
        inv_I = row.inv_IA;
        dw = row.dwA;
        ds = row.dsA;
    } else if (ent_idx == 1) {
        inv_I = row.inv_IB;
        dw = row.dwB;
        ds = row.dsB;
    } else {
        inv_I = row.inv_IC;
        dw = row.dwC;
        ds = row.dsC;
    }

    if (ds) {
        // Split impulse in a spin component and an angular component.
        auto imp = inv_I * row.J[idx_J] * impulse;
        auto spin_imp = dot(row.spin_axis[ent_idx], imp);
        *ds += spin_imp;
        // Subtract spin impulse to obtain angular impulse.
        *dw += imp - row.spin_axis[ent_idx] * spin_imp;
    } else {
        *dw += inv_I * row.J[idx_J] * impulse;
    }
}

void apply_impulse(scalar impulse, constraint_row &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, row, 0);
    apply_angular_impulse(impulse, row, 1);
}

void warm_start(constraint_row &row) {
    apply_impulse(row.impulse, row);
}

void prepare_row3(constraint_row &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &linvelB, const vector3 &linvelC,
                 const vector3 &angvelA, const vector3 &angvelB, const vector3 &angvelC) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]) +
                     dot(row.J[4], row.J[4]) * row.inv_mC +
                     dot(row.inv_IC * row.J[5], row.J[5]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB) +
                  dot(row.J[4], linvelC) +
                  dot(row.J[5], angvelC);

    auto restitution = restitution_curve(options.restitution, relvel);
    row.rhs = -(options.error * options.erp + relvel * (1 + restitution));
}

void apply_impulse3(scalar impulse, constraint_row &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;
    *row.dvC += row.inv_mC * row.J[4] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, row, 0);
    apply_angular_impulse(impulse, row, 1);
    apply_angular_impulse(impulse, row, 2);
}

void warm_start3(constraint_row &row) {
    apply_impulse3(row.impulse, row);
}

}
