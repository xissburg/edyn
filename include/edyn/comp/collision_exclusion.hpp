#ifndef EDYN_COMP_COLLISION_EXCLUSION_HPP
#define EDYN_COMP_COLLISION_EXCLUSION_HPP

#include "edyn/util/array.hpp"
#include <array>
#include <cstdint>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

/**
 * @brief Collision exclusion list. Collisions between a rigid body which has
 * one of these and any of the other rigid bodies contained in the entity
 * array will be ignored.
 */
struct collision_exclusion {
    static constexpr unsigned max_exclusions = 16;
    std::array<entt::entity, max_exclusions> entity =
        make_array<max_exclusions>(entt::entity{entt::null});

    unsigned num_entities() const {
        unsigned i = 0;
        for (; i < max_exclusions; ++i) {
            if (entity[i] == entt::null) {
                break;
            }
        }
        return i;
    }
};

}

#endif // EDYN_COMP_COLLISION_EXCLUSION_HPP
