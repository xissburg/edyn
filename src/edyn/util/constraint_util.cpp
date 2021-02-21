#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/parallel/entity_graph.hpp"

namespace edyn {

namespace internal {
    void pre_make_constraint(entt::entity entity, entt::registry &registry, 
                            entt::entity body0, entt::entity body1, bool is_graph_edge) {

        registry.emplace<procedural_tag>(entity);

        // If the constraint is not a graph edge (e.g. when it's a `contact_constraint`
        // in a contact manifold), it means it is handled as a child of another entity
        // that is a graph edge and thus creating an edge for this would be redundant.
        if (is_graph_edge) {
            auto node_index0 = registry.get<graph_node>(body0).node_index;
            auto node_index1 = registry.get<graph_node>(body1).node_index;
            auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
            registry.emplace<graph_edge>(entity, edge_index);
            registry.emplace<island_container>(entity);
        }

        auto &constraint_dirty = registry.get_or_emplace<dirty>(entity)
            .set_new()
            .created<procedural_tag, constraint>();

        if (is_graph_edge) {
            constraint_dirty.created<graph_edge>();
        }
    }
}

void limit_dirty_to_island_of_procedural(entt::registry &registry, entt::entity ent0, entt::entity ent1) {
    if (!registry.has<procedural_tag>(ent0)) {
        EDYN_ASSERT(registry.has<procedural_tag>(ent1));
        auto *container = registry.try_get<island_container>(ent1);
        if (container && !container->entities.empty()) {
            registry.get_or_emplace<dirty>(ent0).islands(*container->entities.begin());
        }
    }

    if (!registry.has<procedural_tag>(ent1)) {
        EDYN_ASSERT(registry.has<procedural_tag>(ent0));
        auto *container = registry.try_get<island_container>(ent0);
        if (container && !container->entities.empty()) {
            registry.get_or_emplace<dirty>(ent1).islands(*container->entities.begin());
        }
    }
}

entt::entity add_constraint_row(entt::entity entity, constraint &con, entt::registry &registry, int priority) {
    EDYN_ASSERT(con.num_rows() + 1 < max_constraint_rows);
    EDYN_ASSERT(con.row[con.num_rows()] == entt::null);

    auto row_entity = registry.create();
    con.row[con.num_rows()] = row_entity;

    auto &row = registry.emplace<constraint_row>(row_entity, con.body);
    row.priority = priority;

    registry.emplace<constraint_row_data>(row_entity);
    registry.emplace<procedural_tag>(row_entity);

    registry.get_or_emplace<dirty>(row_entity)
        .set_new()
        .created<procedural_tag, constraint_row, constraint_row_data>();

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
    registry.emplace<contact_manifold>(manifold_entity, body0, body1, separation_threshold);

    auto node_index0 = registry.get<graph_node>(body0).node_index;
    auto node_index1 = registry.get<graph_node>(body1).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(manifold_entity, node_index0, node_index1);
    registry.emplace<graph_edge>(manifold_entity, edge_index);
    registry.emplace<island_container>(manifold_entity);

    registry.get_or_emplace<dirty>(manifold_entity)
        .set_new()
        .created<procedural_tag, 
                 graph_edge, 
                 contact_manifold>();

    limit_dirty_to_island_of_procedural(registry, body0, body1);
}

void set_constraint_enabled(entt::entity entity, entt::registry &registry, bool enabled) {
    auto& con = registry.get<constraint>(entity);
    auto num_rows = con.num_rows();
    
    if (enabled) {
        registry.remove<disabled_tag>(entity);

        for (size_t i = 0; i < num_rows; ++i) {
            registry.remove<disabled_tag>(con.row[i]);
        }
    } else {
        registry.emplace_or_replace<disabled_tag>(entity);

        for (size_t i = 0; i < num_rows; ++i) {
            registry.emplace_or_replace<disabled_tag>(con.row[i]);
        }
    }
}

scalar get_effective_mass(const constraint_row_data &row) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

}