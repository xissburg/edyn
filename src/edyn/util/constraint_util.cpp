#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/parallel/component_index_source.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/material_mixing.hpp"

namespace edyn {

namespace internal {
    bool pre_make_constraint(entt::entity entity, entt::registry &registry,
                             entt::entity body0, entt::entity body1) {
        // Multiple constraints of different types can be assigned to the same
        // entity. If this entity already has a graph edge, just do a few
        // consistency checks.
        if (registry.any_of<graph_edge>(entity)) {
        #if defined(EDYN_DEBUG) && !defined(EDYN_DISABLE_ASSERT)
            auto &edge = registry.get<graph_edge>(entity);
            auto [ent0, ent1] = registry.ctx<entity_graph>().edge_node_entities(edge.edge_index);
            EDYN_ASSERT(ent0 == body0 && ent1 == body1);
        #endif
            return false;
        }

        auto node_index0 = registry.get<graph_node>(body0).node_index;
        auto node_index1 = registry.get<graph_node>(body1).node_index;
        auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
        registry.emplace<graph_edge>(entity, edge_index);

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

    auto &dirty = registry.get_or_emplace<edyn::dirty>(manifold_entity);
    dirty.set_new().created<contact_manifold, contact_manifold_events>();

    if (registry.any_of<continuous_contacts_tag>(body0) ||
        registry.any_of<continuous_contacts_tag>(body1)) {

        auto &settings = registry.ctx<edyn::settings>();
        registry.emplace<continuous>(manifold_entity).insert(settings.index_source->index_of<edyn::contact_manifold>());
        dirty.created<continuous>();
    }

    auto material_view = registry.view<material>();

    // Only create contact constraint if bodies have material.
    if (!material_view.contains(body0) || !material_view.contains(body1)) {
        // If not, emplace a null constraint to ensure an edge will exist in
        // the entity graph.
        make_constraint<null_constraint>(manifold_entity, registry, body0, body1);
        return;
    }

    auto &material0 = material_view.get<material>(body0);
    auto &material1 = material_view.get<material>(body1);

    auto &material_table = registry.ctx<material_mix_table>();
    auto restitution = scalar(0);
    auto stiffness = scalar(0);
    auto damping = scalar(0);

    if (auto *material = material_table.try_get({material0.id, material1.id})) {
        restitution = material->restitution;
        stiffness = material->stiffness;
        damping = material->damping;
    } else {
        restitution = material_mix_restitution(material0.restitution, material1.restitution);
        stiffness = material_mix_stiffness(material0.stiffness, material1.stiffness);
        damping = material_mix_damping(material0.damping, material1.damping);
    }

    if (restitution > EDYN_EPSILON) {
        registry.emplace<contact_manifold_with_restitution>(manifold_entity);
        dirty.created<contact_manifold_with_restitution>();
    }

    // Assign contact constraint to manifold.
    auto *tire0 = registry.try_get<tire_material>(body0);
    auto *tire1 = registry.try_get<tire_material>(body1);
    auto *tire = tire0 ? tire0 : tire1;

    if (tire) {
        // Contact patch is always a soft contact since it needs deflection
        // to generate friction forces.
        EDYN_ASSERT(stiffness < large_scalar);

        auto &contact = make_constraint<contact_patch_constraint>(manifold_entity, registry, body0, body1);
        contact.m_normal_stiffness = stiffness;
        contact.m_normal_damping = damping;
        contact.m_speed_sensitivity = tire->speed_sensitivity;
        contact.m_load_sensitivity = tire->load_sensitivity;
        contact.m_lat_tread_stiffness = tire->lat_tread_stiffness;
        contact.m_lon_tread_stiffness = tire->lon_tread_stiffness;
        initialize_contact_patch_constraint(registry, manifold_entity);
    } else {
        make_constraint<contact_constraint>(manifold_entity, registry, body0, body1);
    }
}

void swap_manifold(contact_manifold &manifold) {
    std::swap(manifold.body[0], manifold.body[1]);

    manifold.each_point([] (contact_point &cp) {
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
    if (row.num_entities == 2) {
        auto J = std::array<vector3, 4>{row.J[0], row.J[1], row.J[2], row.J[3]};
        return get_effective_mass(J, row.inv_mA, row.inv_IA, row.inv_mB, row.inv_IB);
    }

    return get_effective_mass(row.J, row.inv_mA, row.inv_IA, row.inv_mB, row.inv_IB, row.inv_mC, row.inv_IC);
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

void prepare_row(constraint_row &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA,
                 const vector3 &linvelB, const vector3 &angvelB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB);

    row.rhs = -(options.error * options.erp + relvel * (1 + options.restitution));
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

    auto delta = inv_I * row.J[idx_J] * impulse;

    if (ds) {
        // Split delta in a spin component and an angular component.
        auto spin = dot(row.spin_axis[ent_idx], delta);
        *ds += spin;
        // Subtract spin to obtain only angular component.
        *dw += delta - row.spin_axis[ent_idx] * spin;
    } else {
        *dw += delta;
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
    row.eff_mass = get_effective_mass(row.J,
                                      row.inv_mA, row.inv_IA,
                                      row.inv_mB, row.inv_IB,
                                      row.inv_mC, row.inv_IC);

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB) +
                  dot(row.J[4], linvelC) +
                  dot(row.J[5], angvelC);

    row.rhs = -(options.error * options.erp + relvel * (1 + options.restitution));
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
