#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/constants.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct contact_manifold {
    std::array<entt::entity, 2> body {entt::null, entt::null};
    scalar separation_threshold;
    std::array<entt::entity, max_contacts> point =
        make_array<max_contacts>(entt::entity{entt::null});

    size_t num_points() const {
        size_t count = 0;
        for (auto e : point) {
            if (e != entt::null) {
                ++count;
            }
        }
        return count;
    }
};

struct contact_manifold_restitution {
    scalar value;
};

}


#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP
