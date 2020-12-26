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
{
    registry.on_construct<AABB>().connect<&broadphase_worker::on_construct_aabb>(*this);
    registry.on_destroy<AABB>().connect<&broadphase_worker::on_destroy_aabb>(*this);
}

void broadphase_worker::on_construct_aabb(entt::registry &registry, entt::entity entity) {
    auto &aabb = registry.get<AABB>(entity);
    auto id = m_tree.create(aabb, entity);
    m_node_map[entity] = id;
}

void broadphase_worker::on_destroy_aabb(entt::registry &registry, entt::entity entity) {
    m_tree.destroy(m_node_map[entity]);
    m_node_map.erase(entity);
}

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

    auto procedural_aabb_view = m_registry->view<AABB, procedural_tag>();
    
    procedural_aabb_view.each([&] (entt::entity entity, AABB &aabb) {
        auto id = m_node_map[entity];
        m_tree.move(id, aabb);
    });

    procedural_aabb_view.each([&] (entt::entity entity, AABB &aabb) {
        auto offset_aabb = aabb.inset(offset);

        m_tree.query(offset_aabb, [&] (dynamic_tree::node_id_t id) {
            auto &node = m_tree.get_node(id);

            if (should_collide(entity, node.entity)) {
                auto &other_aabb = aabb_view.get(node.entity);

                if (intersect(offset_aabb, other_aabb)) {
                    auto p = std::make_pair(entity, node.entity);
                    if (!m_manifold_map.contains(p)) {
                        make_contact_manifold(*m_registry, entity, node.entity, separation_threshold);
                    }
                }
            }

            return true;
        });
    });
}

bool broadphase_worker::should_collide(entt::entity e0, entt::entity e1) const {
    if (e0 == e1) {
        return false;
    }

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