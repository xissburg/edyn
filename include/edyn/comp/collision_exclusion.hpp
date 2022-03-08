#ifndef EDYN_COMP_COLLISION_EXCLUSION_HPP
#define EDYN_COMP_COLLISION_EXCLUSION_HPP

#include <array>
#include <cstdint>
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Collision exclusion list. Collisions between a rigid body which has
 * one of these and any of the other rigid bodies contained in the entity
 * array will be ignored.
 */
struct collision_exclusion {
    static constexpr uint8_t max_exclusions = 16;
    uint8_t num_entities {0};
    std::array<entt::entity, max_exclusions> entity;
};

}

#endif // EDYN_COMP_COLLISION_EXCLUSION_HPP
