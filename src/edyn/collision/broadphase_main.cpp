#include "edyn/collision/broadphase_main.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>
#include <vector>

#include <iostream>

namespace edyn {

broadphase_main::broadphase_main(entt::registry &reg)
    : m_registry(&reg)
{
    reg.on_construct<contact_manifold>().connect<&broadphase_main::on_construct_contact_manifold>(*this);
    reg.on_destroy<contact_manifold>().connect<&broadphase_main::on_destroy_contact_manifold>(*this);
}

void broadphase_main::on_construct_contact_manifold(entt::registry &reg, entt::entity ent) {
    auto &manifold = reg.get<contact_manifold>(ent);
    // Cache the pair for quick look up.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    m_manifold_map[p] = ent;
    m_manifold_map[q] = ent;
}

void broadphase_main::on_destroy_contact_manifold(entt::registry &reg, entt::entity ent) {
    auto &manifold = reg.get<contact_manifold>(ent);
    // Cleanup cached info.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    m_manifold_map.erase(p);
    m_manifold_map.erase(q);
}

void broadphase_main::update() {
    // Search for new AABB intersections and create manifolds.
    // A higher threshold is used in the main broadphase to create contact 
    // manifolds between different islands a little earlier and decrease the
    // probability they'll arrive in the corresponding island worker when the
    // AABBs are already intersecting.
    const auto threshold = contact_breaking_threshold * 4;
    const auto offset = vector3_one * -threshold;
    const auto separation_threshold = threshold * 1.3;

    auto aabb_view = m_registry->view<AABB>();
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
                if (!m_manifold_map.count(p)) {
                    make_contact_manifold(*m_registry, e0, e1, separation_threshold);
                }
            }
        }
    }
}

bool broadphase_main::should_collide(entt::entity e0, entt::entity e1) const {
    // Only consider nodes that reside in different islands.
    auto &container0 = m_registry->get<island_container>(e0);
    auto &container1 = m_registry->get<island_container>(e1);

    // One of the entities must be dynamic.
    if (m_registry->has<dynamic_tag>(e0)) {
        // Dynamic entities are assigned to only one island.
        auto island_entity = container0.entities.front();
        for (auto other_island_entity : container1.entities) {
            if (other_island_entity == island_entity) {
                return false;
            }
        }
    } else if (m_registry->has<dynamic_tag>(e1)) {
        auto island_entity = container1.entities.front();
        for (auto other_island_entity : container0.entities) {
            if (other_island_entity == island_entity) {
                return false;
            }
        }
    } else {
        return false;
    }

    auto view = m_registry->view<const collision_filter>();
    auto &filter0 = view.get(e0);
    auto &filter1 = view.get(e1);
    return ((filter0.group & filter1.mask) > 0) && 
           ((filter1.group & filter0.mask) > 0);
}

}