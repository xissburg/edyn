#ifndef EDYN_UTIL_CONSTRAINT_HPP
#define EDYN_UTIL_CONSTRAINT_HPP

#include <entt/entt.hpp>
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"

namespace edyn {

template<typename T> inline
void make_constraint(entt::entity entity, entt::registry &registry, T&& con, 
                     entt::entity ent0, entt::entity ent1) {
    registry.emplace<island_node>(entity, true, std::vector<entt::entity>{ent0, ent1});
    registry.emplace<constraint>(entity, std::array<entt::entity, 2>{ent0, ent1}, std::forward<T>(con));
    
    auto &node0 = registry.get<island_node>(ent0);
    node0.entities.push_back(entity);
    
    auto &node1 = registry.get<island_node>(ent1);
    node1.entities.push_back(entity);

    registry.get_or_emplace<island_node_dirty>(ent0).indexes.push_back(entt::type_index<island_node>::value());
    registry.get_or_emplace<island_node_dirty>(ent1).indexes.push_back(entt::type_index<island_node>::value());
    registry.get_or_emplace<island_node_dirty>(entity).indexes.push_back(entt::type_index<island_node>::value());
    registry.get_or_emplace<island_node_dirty>(entity).indexes.push_back(entt::type_index<constraint>::value());
}

template<typename T> inline
entt::entity make_constraint(entt::registry &registry, T&& con, 
                             entt::entity ent0, entt::entity ent1) {
    auto ent = registry.create();
    make_constraint<T>(ent, registry, std::forward<T>(con), ent0, ent1);
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

entt::entity add_constraint_row(entt::entity, constraint &, entt::registry &, int priority = 0);

/**
 * Enables/disables a constraint and its rows.
 */
void set_constraint_enabled(entt::entity, entt::registry &, bool enabled);

scalar get_effective_mass(const constraint_row &, 
                          const mass_inv &inv_mA, const inertia_world_inv &inv_IA,
                          const mass_inv &inv_mB, const inertia_world_inv &inv_IB);

}

#endif // EDYN_UTIL_CONSTRAINT_HPP