#ifndef EDYN_UTIL_CONSTRAINT_HPP
#define EDYN_UTIL_CONSTRAINT_HPP

#include <entt/entt.hpp>
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"

namespace edyn {

template<typename T, typename... Entity>
entt::entity make_constraint(entt::registry &registry, T&& c, Entity... entities) {
    auto ent = registry.create();
    registry.assign<relation>(ent, std::forward<Entity>(entities)...);
    registry.assign<constraint>(ent, std::forward<T>(c));
    return ent;
}

}

#endif // EDYN_UTIL_CONSTRAINT_HPP