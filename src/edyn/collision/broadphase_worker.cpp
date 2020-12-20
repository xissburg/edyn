#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/math/constants.hpp"
#include <entt/entt.hpp>
#include <vector>

namespace edyn {

broadphase_worker::broadphase_worker(entt::registry &registry)
    : m_registry(&registry)
    , m_manifold_map(registry)
{}

void broadphase_worker::update() {
    auto aabb_view = m_registry->view<AABB>();
    auto manifold_view = m_registry->view<contact_manifold>();

    // Destroy manifolds of pairs that are not intersecting anymore.
    manifold_view.each([&] (auto ent, contact_manifold &manifold) {
        auto &b0 = aabb_view.get<AABB>(manifold.body[0]);
        auto &b1 = aabb_view.get<AABB>(manifold.body[1]);
        const auto separation_offset = vector3_one * -manifold.separation_threshold;

        if (!intersect(b0.inset(separation_offset), b1)) {
            m_registry->destroy(ent);
        }
    });

    // Search for new AABB intersections and create manifolds.
    const auto offset = vector3_one * -contact_breaking_threshold;
    const auto separation_threshold = contact_breaking_threshold * 4 * 1.3;
    auto it = aabb_view.begin();
    const auto it_end = aabb_view.end();

    for (; it != it_end; ++it) {
        auto e0 = *it;
        auto &b0 = aabb_view.get(e0);

        for (auto it1 = it + 1; it1 != it_end; ++it1) {
            auto e1 = *it1;

            if (!should_collide(e0, e1)) {
                continue;
            }

            auto &b1 = aabb_view.get(e1);

            if (intersect(b0.inset(offset), b1)) {
                auto p = std::make_pair(e0, e1);
                if (!m_manifold_map.contains(p)) {
                    make_contact_manifold(*m_registry, e0, e1, separation_threshold);
                }
            }
        }
    }
}

bool broadphase_worker::should_collide(entt::entity e0, entt::entity e1) const {
    if ((m_registry->has<static_tag>(e0) || m_registry->has<kinematic_tag>(e0)) &&
        (m_registry->has<static_tag>(e1) || m_registry->has<kinematic_tag>(e1))) {
        return false;
    }

    auto view = m_registry->view<const collision_filter>();
    auto &filter0 = view.get(e0);
    auto &filter1 = view.get(e1);
    return ((filter0.group & filter1.mask) > 0) && 
           ((filter1.group & filter0.mask) > 0);
}

}