#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include <vector>
#include <unordered_set>
#include <memory>
#include <entt/fwd.hpp>

namespace edyn {

/**
 * @brief An _island_ is a set of entities that are connected through relations.
 */
struct island {
    // All entities in this island.
    std::vector<entt::entity> entities;

    // The step when all entities in this island reached a speed lower than the
    // threshold. If the speed continues below the threshold for more than a
    // certain period of time, the island is put to sleep.
    uint64_t sleep_step {UINT64_MAX};
};

struct island_timestamp {
    double value;
};

/**
 * @brief An _island node_ is an entity that belongs in one or more islands.
 * It contains a list of the entities it is connected with. A procedural
 * node has its state calculated by the physics simulation (e.g. a dynamic
 * rigid body) and thus can only be present in one island, plus if there is
 * a path connecting two procedural nodes in the graph, they have to be in
 * the same island. A non-procedural node does not have its state affected by
 * the physics simulation (i.e. a static or kinematic rigid body or any 
 * external entity), and that means it can be present in multiple islands
 * at the same time, which means that islands can intersect at non-procedural
 * nodes. Also, when calculating procedural node connectivity, non-procedural
 * nodes do not generate paths, i.e. it acts as a wall that you cannot walk
 * through when generating islands.
 */
struct island_node {
    std::vector<entt::entity> entities;
};

struct island_container {
    std::vector<entt::entity> entities;
};

struct island_node_dirty {
    std::unordered_set<entt::id_type> indexes;
};

}

#endif // EDYN_COMP_ISLAND_HPP