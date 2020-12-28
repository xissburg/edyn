#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/tree_view.hpp"
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
    registry.on_destroy<tree_node_id_t>().connect<&broadphase_worker::on_destroy_node_id>(*this);
}

void broadphase_worker::on_construct_aabb(entt::registry &registry, entt::entity entity) {
    // Perform initialization later when the entity is fully constructed.
    m_new_aabb_entities.push_back(entity);
}

void broadphase_worker::on_destroy_node_id(entt::registry &registry, entt::entity entity) {
    auto id = registry.get<tree_node_id_t>(entity);

    if (registry.has<procedural_tag>(entity)) {
        m_tree.destroy(id);
    } else {
        m_np_tree.destroy(id);
    }
}

void broadphase_worker::init_new_aabb_entities() {
    if (m_new_aabb_entities.empty()) {
        return;
    }

    for (auto entity : m_new_aabb_entities) {
        auto &aabb = m_registry->get<AABB>(entity);
        tree_node_id_t id = m_registry->has<procedural_tag>(entity) ?
            m_tree.create(aabb, entity) :
            m_np_tree.create(aabb, entity);
        m_registry->emplace<tree_node_id_t>(entity, id);
    }

    m_new_aabb_entities.clear();
}

void broadphase_worker::update() {
    init_new_aabb_entities();

    auto aabb_view = m_registry->view<AABB>();
    auto manifold_view = m_registry->view<contact_manifold>();

    // Destroy manifolds of pairs whose AABBs are not intersecting anymore.
    manifold_view.each([&] (entt::entity entity, contact_manifold &manifold) {
        auto &b0 = aabb_view.get<AABB>(manifold.body[0]);
        auto &b1 = aabb_view.get<AABB>(manifold.body[1]);
        const auto separation_offset = vector3_one * -manifold.separation_threshold;

        if (!intersect(b0.inset(separation_offset), b1)) {
            m_registry->destroy(entity);
        }
    });

    // Update AABBs of procedural nodes in the dynamic tree.
    auto proc_aabb_node_view = m_registry->view<tree_node_id_t, AABB, procedural_tag>();
    proc_aabb_node_view.each([&] (tree_node_id_t node_id, AABB &aabb) {
        m_tree.move(node_id, aabb);
    });

    // Update kinematic AABBs in non-procedural tree.
    // TODO: only do this for kinematic entities that had their AABB updated.
    auto kinematic_aabb_node_view = m_registry->view<tree_node_id_t, AABB, kinematic_tag>();
    kinematic_aabb_node_view.each([&] (tree_node_id_t node_id, AABB &aabb) {
        m_np_tree.move(node_id, aabb);
    });

    // Search for new AABB intersections and create manifolds.
    auto aabb_proc_view = m_registry->view<AABB, procedural_tag>();
    aabb_proc_view.each([&] (entt::entity entity, AABB &aabb) {
        auto offset_aabb = aabb.inset(m_aabb_offset);
        collide_tree(m_tree, entity, offset_aabb);
        collide_tree(m_np_tree, entity, offset_aabb);
    });
}

void broadphase_worker::collide_tree(const dynamic_tree &tree, entt::entity entity, const AABB &offset_aabb) {
    auto aabb_view = m_registry->view<AABB>();

    tree.query(offset_aabb, [&] (tree_node_id_t id) {
        auto &node = tree.get_node(id);

        if (should_collide(entity, node.entity) && !m_manifold_map.contains(entity, node.entity)) {
            auto &other_aabb = aabb_view.get(node.entity);

            if (intersect(offset_aabb, other_aabb)) {
                make_contact_manifold(*m_registry, entity, node.entity, m_separation_threshold);
            }
        }
    });
}

bool broadphase_worker::should_collide(entt::entity e0, entt::entity e1) const {
    if (e0 == e1) {
        return false;
    }

    auto view = m_registry->view<const collision_filter>();
    auto &filter0 = view.get(e0);
    auto &filter1 = view.get(e1);
    return ((filter0.group & filter1.mask) > 0) && 
           ((filter1.group & filter0.mask) > 0);
}

tree_view broadphase_worker::view() const {
    return m_tree.view();
}

}
