#ifndef EDYN_UTIL_CONSTRAINT_HPP
#define EDYN_UTIL_CONSTRAINT_HPP

#include <entt/entt.hpp>
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"

namespace edyn {

template<typename T>
entt::entity make_constraint(entt::registry &registry, T&& c, entt::entity ent0, entt::entity ent1, entt::entity ent2 = entt::null) {
    auto ent = registry.create();
    registry.assign<relation>(ent, ent0, ent1, ent2);
    registry.assign<constraint>(ent, std::forward<T>(c));
    return ent;
}

template<typename T>
T& get_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    return std::get<T>(con.var);
}

}

#endif // EDYN_UTIL_CONSTRAINT_HPP