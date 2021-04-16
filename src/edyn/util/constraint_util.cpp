#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/constraint_row.hpp"

namespace edyn {

namespace internal {
    void pre_make_constraint(entt::entity entity, entt::registry &registry, 
                             entt::entity body0, entt::entity body1, bool is_graph_edge) {
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
    }
}

entt::entity make_contact_manifold(entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    auto manifold_entity = registry.create();
    make_contact_manifold(manifold_entity, registry, body0, body1, separation_threshold);
    return manifold_entity;
}

void make_contact_manifold(entt::entity manifold_entity, entt::registry &registry, entt::entity body0, entt::entity body1, scalar separation_threshold) {
    EDYN_ASSERT(registry.valid(body0) && registry.valid(body1));
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

void apply_impulse(scalar impulse, constraint_row &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    *row.dwA += row.inv_IA * row.J[1] * impulse;
    *row.dwB += row.inv_IB * row.J[3] * impulse;
}

void warm_start(constraint_row &row) {
    apply_impulse(row.impulse, row);
}

}