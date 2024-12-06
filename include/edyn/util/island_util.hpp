#ifndef EDYN_UTIL_ISLAND_UTIL_HPP
#define EDYN_UTIL_ISLAND_UTIL_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/replication/entity_map.hpp"

namespace edyn {

static constexpr auto exclude_sleeping_disabled = entt::exclude_t<sleeping_tag, disabled_tag>{};

void remove_sleeping_tag_from_island(entt::registry &registry,
                                     entt::entity island_entity,
                                     const edyn::island &island);

entt::sparse_set collect_islands_from_residents(entt::registry &registry, const std::vector<entt::entity> &entities);
entt::sparse_set collect_islands_from_residents(entt::registry &registry, const entt::sparse_set &entities);

void wake_up_island(entt::registry &registry, entt::entity island_entity);

void wake_up_island_residents(entt::registry &registry, const std::vector<entt::entity> &entities);

void wake_up_island_residents(entt::registry &registry, const entt::sparse_set &entities);

void wake_up_island_resident(entt::registry &registry, entt::entity entity);

void wake_up_island_residents(entt::registry &registry, const std::vector<entt::entity> &entities,
                              const entity_map &emap);

void wake_up_island_residents(entt::registry &registry, const entt::sparse_set &entities,
                              const entity_map &emap);

}

#endif // EDYN_UTIL_ISLAND_UTIL_HPP
