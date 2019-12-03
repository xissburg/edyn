#ifndef EDYN_DYNAMICS_ISLAND_UTIL_HPP
#define EDYN_DYNAMICS_ISLAND_UTIL_HPP

#include <entt/fwd.hpp>
#include "edyn/comp/relation.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

constexpr auto exclude_sleeping = entt::exclude_t<sleeping_tag>{};

void wakeup_island(entt::entity, entt::registry &);

void put_islands_to_sleep(entt::registry &, uint64_t step, scalar dt);

void island_on_construct_relation(entt::entity, entt::registry &, relation &);

void island_on_destroy_relation(entt::entity, entt::registry &);

}

#endif // EDYN_DYNAMICS_ISLAND_UTIL_HPP