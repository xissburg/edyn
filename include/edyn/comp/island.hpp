#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include "edyn/util/entity_set.hpp"

namespace edyn {

/**
 * @brief An _island_ is a set of entities that can affect one another,
 * usually through constraints.
 */
struct island {

};

struct island_timestamp {
    double value;
};

/**
 * @brief An _island node_ is an entity that belongs in one or more islands.
 * It contains a set of the entities it is connected with. A procedural
 * node has its state calculated by the physics simulation (e.g. a dynamic
 * rigid body) and thus can only be present in one island, plus if there is
 * a path connecting two procedural nodes in the graph, they have to be in
 * the same island. A non-procedural node does not have its state affected by
 * the physics simulation (i.e. a static or kinematic rigid body or any 
 * external entity), and that means it can be present in multiple islands
 * at the same time, which means that islands can intersect at non-procedural
 * nodes. Also, when calculating procedural node connectivity, non-procedural
 * nodes do not generate paths, i.e. it acts as a wall that you cannot walk
 * through when generating islands via the connectivity graph.
 */
struct island_node {
    entity_set entities;
};

/**
 * @brief An _island container_ holds a set of island entities where an 
 * _island node_ belongs.
 */
struct island_container {
    entity_set entities;
};

/**
 * @brief Island nodes can act as parents and have child entities, effectively
 * instantiating a tree of sub entities within a graph node where the children
 * could not exist without the parent. This allows the graph search for island
 * generation to be optimized since the child entities won't have to be visited.
 * Every child must have an `island_node_child` component. Only the root of the
 * tree should have a `island_node` component.
 */
struct island_node_parent {
    entity_set children;
};

/**
 * @brief A child of another node. The parent node must have an `island_node_parent`
 * component which references the child back.
 */
struct island_node_child {
    entt::entity parent;
};

}

#endif // EDYN_COMP_ISLAND_HPP