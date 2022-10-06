#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tree_resident.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

struct island_tree_resident {
    tree_node_id_t id;
};

broadphase::broadphase(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<AABB>().connect<&broadphase::on_construct_aabb>(*this);
    registry.on_destroy<tree_resident>().connect<&broadphase::on_destroy_tree_resident>(*this);
    registry.on_construct<island_AABB>().connect<&broadphase::on_construct_island_aabb>(*this);
    registry.on_destroy<island_tree_resident>().connect<&broadphase::on_destroy_island_tree_resident>(*this);

    // The `should_collide_func` function will be invoked in parallel when
    // running broadphase in parallel, in the call to `broadphase::collide_tree_async`.
    // Avoid multi-threading issues by pre-allocating the pools that will be
    // needed in `should_collide_func`.
    static_cast<void>(registry.storage<collision_filter>());
    static_cast<void>(registry.storage<collision_exclusion>());
}

void broadphase::on_construct_aabb(entt::registry &, entt::entity entity) {
    // Perform initialization later when the entity is fully constructed.
    m_new_aabb_entities.push_back(entity);
}

void broadphase::on_destroy_tree_resident(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<tree_resident>(entity);

    if (node.procedural) {
        m_tree.destroy(node.id);
    } else {
        m_np_tree.destroy(node.id);
    }
}

void broadphase::on_construct_island_aabb(entt::registry &registry, entt::entity entity) {
    auto &aabb = registry.get<island_AABB>(entity);
    tree_node_id_t id = m_island_tree.create(aabb, entity);
    registry.emplace<island_tree_resident>(entity, id);
}

void broadphase::on_destroy_island_tree_resident(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<island_tree_resident>(entity);
    m_island_tree.destroy(node.id);
}

void broadphase::init_new_aabb_entities() {
    if (m_new_aabb_entities.empty()) {
        return;
    }

    auto aabb_view = m_registry->view<AABB>();
    auto procedural_view = m_registry->view<procedural_tag>();

    for (auto entity : m_new_aabb_entities) {
        // Entity might've been destroyed, thus skip it.
        if (!m_registry->valid(entity)) continue;

        auto &aabb = aabb_view.get<AABB>(entity);
        bool procedural = procedural_view.contains(entity);
        auto &tree = procedural ? m_tree : m_np_tree;
        tree_node_id_t id = tree.create(aabb, entity);
        m_registry->emplace<tree_resident>(entity, id, procedural);
    }

    m_new_aabb_entities.clear();
}

void broadphase::move_aabbs() {
    // Update AABBs of procedural nodes in the dynamic tree.
    auto proc_aabb_node_view = m_registry->view<tree_resident, AABB, procedural_tag>(exclude_sleeping_disabled);
    proc_aabb_node_view.each([&](tree_resident &node, AABB &aabb) {
        m_tree.move(node.id, aabb);
    });

    // Update kinematic AABBs in non-procedural tree.
    // TODO: only do this for kinematic entities that had their AABB updated.
    auto kinematic_aabb_node_view = m_registry->view<tree_resident, AABB, kinematic_tag>(exclude_sleeping_disabled);
    kinematic_aabb_node_view.each([&](tree_resident &node, AABB &aabb) {
        m_np_tree.move(node.id, aabb);
    });

    auto island_aabb_node_view = m_registry->view<island_tree_resident, island_AABB>(entt::exclude_t<sleeping_tag>{});
    island_aabb_node_view.each([&](island_tree_resident &node, island_AABB &aabb) {
        m_island_tree.move(node.id, aabb);
    });
}

void broadphase::destroy_separated_manifolds() {
    auto aabb_view = m_registry->view<AABB>();
    auto manifold_view = m_registry->view<contact_manifold>(exclude_sleeping_disabled);

    // Destroy manifolds of pairs whose AABBs are not intersecting anymore.
    manifold_view.each([&](entt::entity entity, contact_manifold &manifold) {
        auto [b0] = aabb_view.get(manifold.body[0]);
        auto [b1] = aabb_view.get(manifold.body[1]);
        const auto separation_offset = vector3_one * -manifold.separation_threshold;

        if (!intersect(b0.inset(separation_offset), b1)) {
            m_registry->destroy(entity);
        }
    });
}

void broadphase::collide_tree(const dynamic_tree &tree, entt::entity entity,
                              const AABB &offset_aabb) const {
    auto aabb_view = m_registry->view<AABB>();
    auto &settings = m_registry->ctx().at<edyn::settings>();
    auto &manifold_map = m_registry->ctx().at<contact_manifold_map>();

    tree.query(offset_aabb, [&](tree_node_id_t id) {
        auto &node = tree.get_node(id);
        auto collides = (*settings.should_collide_func)(*m_registry, entity, node.entity);

        if (collides && !manifold_map.contains(entity, node.entity)) {
            auto [other_aabb] = aabb_view.get(node.entity);

            if (intersect(offset_aabb, other_aabb)) {
                make_contact_manifold(*m_registry, entity, node.entity, m_separation_threshold);
            }
        }
    });
}

void broadphase::collide_tree_async(const dynamic_tree &tree, entt::entity entity,
                                    const AABB &offset_aabb, size_t result_index) {
    auto aabb_view = m_registry->view<AABB>();
    auto &settings = m_registry->ctx().at<edyn::settings>();

    tree.query(offset_aabb, [&](tree_node_id_t id) {
        auto &node = tree.get_node(id);

        if ((*settings.should_collide_func)(*m_registry, entity, node.entity)) {
            auto [other_aabb] = aabb_view.get(node.entity);

            if (intersect(offset_aabb, other_aabb)) {
                m_pair_results[result_index].emplace_back(entity, node.entity);
            }
        }
    });
}

void broadphase::update(bool mt) {
    init_new_aabb_entities();
    destroy_separated_manifolds();
    move_aabbs();

    // Search for new AABB intersections and create manifolds.
    auto aabb_proc_view = m_registry->view<AABB, procedural_tag>(exclude_sleeping_disabled);

    if (mt && calculate_view_size(aabb_proc_view) > m_max_sequential_size) {
        collide_parallel();
        finish_collide();
    } else {
        for (auto [entity, aabb] : aabb_proc_view.each()) {
            auto offset_aabb = aabb.inset(m_aabb_offset);
            collide_tree(m_tree, entity, offset_aabb);
            collide_tree(m_np_tree, entity, offset_aabb);
        }
    }
}

void broadphase::collide_parallel() {
    auto aabb_proc_view = m_registry->view<AABB, procedural_tag>(exclude_sleeping_disabled);
    m_pair_results.resize(calculate_view_size(aabb_proc_view));
    auto &dispatcher = job_dispatcher::global();

    auto for_loop_body = [this, aabb_proc_view](entt::entity entity, size_t index) {
        auto &aabb = aabb_proc_view.get<AABB>(entity);
        auto offset_aabb = aabb.inset(m_aabb_offset);
        collide_tree_async(m_tree, entity, offset_aabb, index);
        collide_tree_async(m_np_tree, entity, offset_aabb, index);
    };

    parallel_for_each(dispatcher, aabb_proc_view.begin(), aabb_proc_view.end(), for_loop_body);
}

void broadphase::finish_collide() {
    auto &manifold_map = m_registry->ctx().at<contact_manifold_map>();

    for (auto &pairs : m_pair_results) {
        for (auto &pair : pairs) {
            if (!manifold_map.contains(pair.first, pair.second)) {
                make_contact_manifold(*m_registry, pair.first, pair.second, m_separation_threshold);
            }
        }
        pairs.clear();
    }
}

}
