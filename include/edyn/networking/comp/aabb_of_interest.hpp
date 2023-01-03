#ifndef EDYN_NETWORKING_AABB_OF_INTEREST_HPP
#define EDYN_NETWORKING_AABB_OF_INTEREST_HPP

#include "edyn/comp/aabb.hpp"
#include <entt/signal/sigh.hpp>
#include <entt/entity/sparse_set.hpp>
#include <vector>

namespace edyn {

/**
 * @brief Assigned to each client in the server side. Networked entities which
 * intersect this AABB will be shared with the client.
 */
struct aabb_of_interest {
    // The AABB of interest.
    AABB aabb {vector3_one * -500, vector3_one * 500};

    // Entities in the islands above, including nodes and edges. This is used
    // as a way to tell which entities have entered and exited the AABB.
    entt::sparse_set entities {};

    // Entities that entered and exited the AABB in the last update. These
    // containers are for temporary data storage in the AABB of interest update
    // and so they get cleared up in every update and should not be modified.
    std::vector<entt::entity> entities_entered;
    std::vector<entt::entity> entities_exited;
};

}

#endif // EDYN_NETWORKING_AABB_OF_INTEREST_HPP
