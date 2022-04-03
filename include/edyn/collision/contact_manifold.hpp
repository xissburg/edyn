#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <limits>
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

struct contact_manifold {
    using contact_id_type = unsigned;
    static constexpr auto invalid_id = std::numeric_limits<contact_id_type>::max();

    // Pair of rigid bodies which are touching.
    std::array<entt::entity, 2> body {entt::null, entt::null};

    // If the AABB of one of the bodies inflated by this amount does not
    // intersect the AABB of the other, the manifold will be destroyed.
    // See `edy::broadphase_worker::destroy_separated_manifolds`.
    scalar separation_threshold;

    // Number of contact points in this manifold.
    uint8_t num_points {0};

    // Ids/indices of contact points in this manifold. Only the entries at
    // indices up to `num_points - 1` are valid.
    std::array<contact_id_type, max_contacts> ids;

    // Array of contact points. Must be accessed via the valid indices in
    // the `ids` array.
    std::array<contact_point, max_contacts> point;

    /**
     * @brief Get a contact point by index.
     * @param index Contact point index.
     * @return Contact point with id at the given index.
     */
    auto & get_point(unsigned index) const {
        EDYN_ASSERT(index < num_points);
        return point[ids[index]];
    }

    /*! @copydoc get_point */
    auto & get_point(unsigned index) {
        return const_cast<contact_point &>(std::as_const(*this).get_point(index));
    }

    template<typename Func>
    void each_point(Func func) {
        for (auto i = num_points; i; --i) {
            func(point[ids[i-1]]);
        }
    }

    template<typename Func>
    void each_point(Func func) const {
        for (auto i = num_points; i; --i) {
            func(point[ids[i-1]]);
        }
    }
};

/**
 * Tag assigned to contact manifolds with non-zero restitution.
 */
struct contact_manifold_with_restitution {};

template<typename Archive>
void serialize(Archive &archive, contact_manifold &manifold) {
    archive(manifold.body);
    archive(manifold.separation_threshold);
    archive(manifold.num_points);

    for (unsigned i = 0; i < manifold.num_points; ++i) {
        archive(manifold.ids[i]);
    }

    manifold.each_point([&] (contact_point &cp) {
        archive(cp);
    });
}

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP
