#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include <vector>
#include <memory>
#include <entt/fwd.hpp>

namespace edyn {

class island_worker_context_base;
class buffer_sync_reader;
class message_queue_in_out;

/**
 * @brief An _island_ is a set of entities that are connected through relations.
 */
struct island {
    island_worker_context_base *worker;
    buffer_sync_reader *reader;
    message_queue_in_out *message_queue;
    double timestamp;

    // All entities in this island.
    std::vector<entt::entity> entities;

    // The step when all entities in this island reached a speed lower than the
    // threshold. If the speed continues below the threshold for more than a
    // certain period of time, the island is put to sleep.
    uint64_t sleep_step {UINT64_MAX};
};

/**
 * @brief An _island node_ is an entity that belongs in an island.
 */
struct island_node {
    // The entity where the `island` component can be found.
    entt::entity island_entity;
};

}

#endif // EDYN_COMP_ISLAND_HPP