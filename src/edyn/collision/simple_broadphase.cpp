#include "edyn/collision/simple_broadphase.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/dynamics/island_util.hpp"
#include <entt/entt.hpp>
#include <vector>

namespace edyn {

simple_broadphase::simple_broadphase(entt::registry &reg)
    : m_registry(&reg)
{

}

void simple_broadphase::update() {
    auto view = m_registry->view<const position, const orientation, const shape, AABB>(exclude_global);
    view.each([] (auto, auto &pos, auto &orn, auto &sh, auto &aabb) {
        std::visit([&] (auto &&s) {
            aabb = s.aabb(pos, orn);
        }, sh.var);
    });

    auto aabb_view = m_registry->view<const AABB>();

    auto it = aabb_view.begin();
    const auto it_end = aabb_view.end();

    constexpr auto offset = vector3{-0.1, -0.1, -0.1};

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
                m_intersect_signal.publish(e0, e1);
            }
        }
    }
}

bool simple_broadphase::should_collide(entt::entity e0, entt::entity e1) const {
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