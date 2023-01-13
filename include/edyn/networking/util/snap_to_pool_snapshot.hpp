#ifndef EDYN_NETWORKING_UTIL_SNAP_TO_POOL_SNAPSHOT_HPP
#define EDYN_NETWORKING_UTIL_SNAP_TO_POOL_SNAPSHOT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>

namespace edyn {

struct pool_snapshot;
class entity_map;

void snap_to_pool_snapshot(entt::registry &registry, const entity_map &emap,
                           const std::vector<entt::entity> &entities,
                           const std::vector<pool_snapshot> &pools,
                           bool should_accumulate_discontinuities);

void snap_to_pool_snapshot(entt::registry &registry,
                           const std::vector<entt::entity> &entities,
                           const std::vector<pool_snapshot> &pools,
                           bool should_accumulate_discontinuities);

}

#endif // EDYN_NETWORKING_UTIL_SNAP_TO_POOL_SNAPSHOT_HPP
