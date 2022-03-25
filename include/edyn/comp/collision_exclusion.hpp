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
    using size_type = uint8_t;
    static constexpr size_type max_exclusions = 16;
    std::array<entt::entity, max_exclusions> entity =
        make_array<max_exclusions>(entt::entity{entt::null});

    size_type num_entities() const {
        size_type i = 0;
        for (; i < max_exclusions; ++i) {
            if (entity[i] == entt::null) {
                break;
            }
        }
        return i;
    }
};

template<typename Archive>
void serialize(Archive &archive, collision_exclusion &excl) {
    auto num_entities = excl.num_entities();
    archive(num_entities);
    num_entities = std::min(num_entities, collision_exclusion::max_exclusions);

    for (unsigned i = 0; i < num_entities; ++i) {
        archive(excl.entity[i]);
    }
}

}

#endif // EDYN_COMP_COLLISION_EXCLUSION_HPP
