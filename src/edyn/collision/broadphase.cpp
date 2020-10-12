#include "edyn/collision/broadphase.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/dynamics/island_util.hpp"
#include <entt/entt.hpp>
#include <vector>

namespace edyn {

broadphase::broadphase(entt::registry &reg)
    : m_registry(&reg)
{
    reg.on_construct<contact_manifold>().connect<&broadphase::on_construct_contact_manifold>(*this);
    reg.on_destroy<contact_manifold>().connect<&broadphase::on_destroy_contact_manifold>(*this);
}

void broadphase::on_construct_contact_manifold(entt::registry &reg, entt::entity ent) {
    auto &manifold = reg.get<contact_manifold>(ent);
    // Cache the pair for quick look up.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    m_manifold_map[p] = ent;
    m_manifold_map[q] = ent;
}

void broadphase::on_destroy_contact_manifold(entt::registry &reg, entt::entity ent) {
    auto &manifold = reg.get<contact_manifold>(ent);
    // Cleanup cached info.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    m_manifold_map.erase(p);
    m_manifold_map.erase(q);
}

void broadphase::update() {
    auto view = m_registry->view<const position, const orientation, const shape, AABB>(exclude_global);
    view.each([] (auto, auto &pos, auto &orn, auto &sh, auto &aabb) {
        std::visit([&] (auto &&s) {
            aabb = s.aabb(pos, orn);
        }, sh.var);
    });

    auto aabb_view = m_registry->view<const AABB>();
    auto manifold_view = m_registry->view<contact_manifold>();

    constexpr auto offset = vector3 {
        -contact_breaking_threshold, 
        -contact_breaking_threshold, 
        -contact_breaking_threshold
    };

    // Use slightly higher offset when looking for separation to avoid high
    // frequency creation and destruction of pairs with slight movement.
    constexpr scalar offset_scale = 2;
    constexpr auto separation_offset = vector3 {
        offset.x * offset_scale, offset.y * offset_scale, offset.z * offset_scale};

    // Destroy manifolds of pairs that are not intersecting anymore.
    manifold_view.each([&] (auto ent, contact_manifold &manifold) {
        auto b0 = m_registry->try_get<AABB>(manifold.body[0]);
        auto b1 = m_registry->try_get<AABB>(manifold.body[1]);

        if (b0 && b1 && !intersect(b0->inset(separation_offset), b1->inset(separation_offset))) {
            m_registry->destroy(ent);
        }
    });

    // Search for new AABB intersections and create manifolds.
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

            if (intersect(b0.inset(offset), b1.inset(offset))) {
                auto p = std::make_pair(e0, e1);
                if (!m_manifold_map.count(p)) {
                    auto ent = m_registry->create();
                    m_registry->emplace<contact_manifold>(ent, e0, e1);
                    m_registry->emplace<island_node>(ent, true, std::vector<entt::entity>{e0, e1});
                }
            }
        }
    }
}

bool broadphase::should_collide(entt::entity e0, entt::entity e1) const {
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