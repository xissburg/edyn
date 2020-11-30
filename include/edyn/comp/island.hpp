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
    std::unordered_set<entt::entity> entities;
};

struct island_container {
    std::unordered_set<entt::entity> entities;
};

struct island_node_dirty {
    bool is_new_entity {false};
    std::unordered_set<entt::id_type> created_indexes;
    std::unordered_set<entt::id_type> updated_indexes;
    std::unordered_set<entt::id_type> destroyed_indexes;
};

}

#endif // EDYN_COMP_ISLAND_HPP