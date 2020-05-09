#ifndef EDYN_UTIL_CONSTRAINT_HPP
#define EDYN_UTIL_CONSTRAINT_HPP

#include <entt/entt.hpp>
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"

namespace edyn {

template<typename T> inline
void make_constraint(entt::entity entity, entt::registry &registry, T&& c, 
                     entt::entity ent0, entt::entity ent1, entt::entity ent2 = entt::null) {
    registry.assign<relation>(entity, ent0, ent1, ent2);
    registry.assign<constraint>(entity, std::forward<T>(c));
}

template<typename T> inline
entt::entity make_constraint(entt::registry &registry, T&& c, entt::entity ent0, 
                             entt::entity ent1, entt::entity ent2 = entt::null) {
    auto ent = registry.create();
    make_constraint<T>(ent, registry, std::forward<T>(c), ent0, ent1, ent2);
    return ent;
}

template<typename T> inline
T& get_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    return std::get<T>(con.var);
}

/**
 * Enables/disables a constraint and its rows.
 */
void set_constraint_enabled(entt::entity, entt::registry &, bool enabled);

scalar get_effective_mass(const constraint_row &, 
                          const mass_inv &inv_mA, const inertia_world_inv &inv_IA,
                          const mass_inv &inv_mB, const inertia_world_inv &inv_IB);

}

#endif // EDYN_UTIL_CONSTRAINT_HPP