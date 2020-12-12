#ifndef EDYN_UTIL_CONSTRAINT_HPP
#define EDYN_UTIL_CONSTRAINT_HPP

#include <entt/entt.hpp>
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/dirty.hpp"

namespace edyn {

template<typename T> inline
void make_constraint(entt::entity entity, entt::registry &registry, T&& con, 
                     entt::entity ent0, entt::entity ent1, entt::entity *parent_entity = nullptr) {

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
        registry.emplace<island_node>(entity, entity_set{ent0, ent1});
    }

    // A constraint is a parent of its rows, thus it needs an `island_node_parent`.
    registry.emplace<island_node_parent>(entity);
    registry.emplace<island_container>(entity);
    registry.emplace<constraint>(entity, std::array<entt::entity, 2>{ent0, ent1}, std::forward<T>(con));

    if (!parent_entity) {
        // When there's no parent entity, the constraint is associated with the
        // rigid bodies directly.
        auto &node0 = registry.get<island_node>(ent0);
        node0.entities.insert(entity);

        auto &node1 = registry.get<island_node>(ent1);
        node1.entities.insert(entity);
        
        registry.get_or_emplace<dirty>(ent0).updated<island_node>();
        registry.get_or_emplace<dirty>(ent1).updated<island_node>();
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

template<typename T> inline
entt::entity make_constraint(entt::registry &registry, T&& con, 
                             entt::entity ent0, entt::entity ent1, 
                             entt::entity *parent_entity = nullptr) {
    auto ent = registry.create();
    make_constraint<T>(ent, registry, std::forward<T>(con), ent0, ent1, parent_entity);
    return ent;
}

/**
 * Gets the constraint of type `T` in the `constraint` component assigned
 * to `entity`.
 */
template<typename T> inline
T & get_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    return std::get<T>(con.var);
}

/**
 * Attempts to get a constraint of type `T` in the `constraint` component
 * assigned to `entity`.
 */
template<typename T> inline
T * try_get_constraint(entt::entity entity, entt::registry &registry) {
    auto *con = registry.try_get<constraint>(entity);
    if (con && std::holds_alternative<T>(con->var)) {
        return &std::get<T>(con->var);
    }
    return nullptr;
}

/**
 * Adds one more constraint row to the given constraint. A new entity is created
 * for the row with a `constraint_row` component and it also gets an island node
 * assigned to it which is then associated with the node in the given entity. 
 * The new entity is appended to the row array in the `constraint` and it is 
 * also returned.
 */
entt::entity add_constraint_row(entt::entity, constraint &, entt::registry &, int priority = 0);

entt::entity make_contact_manifold(entt::registry &, entt::entity body0, entt::entity body1,
                                   scalar separation_threshold);
void make_contact_manifold(entt::entity contact_entity, entt::registry &, 
                           entt::entity body0, entt::entity body1, scalar separation_threshold);

/**
 * Enables/disables a constraint and its rows.
 */
void set_constraint_enabled(entt::entity, entt::registry &, bool enabled);

scalar get_effective_mass(const constraint_row &, 
                          const mass_inv &inv_mA, const inertia_world_inv &inv_IA,
                          const mass_inv &inv_mB, const inertia_world_inv &inv_IB);

}

#endif // EDYN_UTIL_CONSTRAINT_HPP