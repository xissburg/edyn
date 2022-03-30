#include "edyn/collision/broadphase_main.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tree_resident.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

broadphase_main::broadphase_main(entt::registry &registry)
    : m_registry(&registry)
{
    // Add tree nodes for islands (which have a `tree_view`) and for static and
    // kinematic entities.
    registry.on_construct<tree_view>().connect<&broadphase_main::on_construct_tree_view>(*this);
    registry.on_construct<static_tag>().connect<&broadphase_main::on_construct_static_kinematic_tag>(*this);
    registry.on_construct<kinematic_tag>().connect<&broadphase_main::on_construct_static_kinematic_tag>(*this);
    registry.on_construct<AABB>().connect<&broadphase_main::on_construct_aabb>(*this);
    registry.on_destroy<tree_resident>().connect<&broadphase_main::on_destroy_tree_resident>(*this);
}

void broadphase_main::on_construct_tree_view(entt::registry &registry, entt::entity entity) {
    EDYN_ASSERT(registry.any_of<island>(entity));

    auto &view = registry.get<tree_view>(entity);
    auto id = m_island_tree.create(view.root_aabb(), entity);
    registry.emplace<tree_resident>(entity, id, true);
}

void broadphase_main::on_construct_static_kinematic_tag(entt::registry &registry, entt::entity entity) {
    if (auto *aabb = registry.try_get<AABB>(entity)) {
        auto id = m_np_tree.create(*aabb, entity);
        registry.emplace<tree_resident>(entity, id, false);
    }
}

void broadphase_main::on_construct_aabb(entt::registry &registry, entt::entity entity) {
    if (registry.any_of<static_tag, kinematic_tag>(entity)) {
        auto &aabb = registry.get<AABB>(entity);
        auto id = m_np_tree.create(aabb, entity);
        registry.emplace<tree_resident>(entity, id, false);
    }
}

void broadphase_main::on_destroy_tree_resident(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<tree_resident>(entity);

    if (node.procedural) {
        m_island_tree.destroy(node.id);
    } else {
        m_np_tree.destroy(node.id);
    }
}

void broadphase_main::update() {
    // Update island AABBs in tree (ignore sleeping islands).
    auto exclude_sleeping = entt::exclude_t<sleeping_tag>{};
    auto tree_view_resident_view = m_registry->view<tree_view, tree_resident>(exclude_sleeping);
    tree_view_resident_view.each([&] (tree_view &tree_view, tree_resident &node) {
        m_island_tree.move(node.id, tree_view.root_aabb());
    });

    // Update kinematic AABBs in tree.
    // TODO: only do this for kinematic entities that had their AABB updated.
    auto kinematic_aabb_node_view = m_registry->view<tree_resident, AABB, kinematic_tag>();
    kinematic_aabb_node_view.each([&] (tree_resident &node, AABB &aabb) {
        m_np_tree.move(node.id, aabb);
    });

    // Search for island pairs with intersecting AABBs, i.e. the AABB of the root
    // node of their trees intersect.
    auto tree_view_not_sleeping_view = m_registry->view<tree_view>(exclude_sleeping);

    std::vector<entt::entity> awake_island_entities;
    for (auto entity : tree_view_not_sleeping_view) {
        awake_island_entities.push_back(entity);
    }

    if (awake_island_entities.empty()) {
        return;
    }

    const auto aabb_view = m_registry->view<AABB>();
    const auto tree_view_view = m_registry->view<tree_view>();
    const auto multi_resident_view = m_registry->view<multi_island_resident>();

    if (awake_island_entities.size() > 1) {
        m_pair_results.resize(awake_island_entities.size());

        parallel_for(size_t{0}, awake_island_entities.size(), [&] (size_t index) {
            auto island_entityA = awake_island_entities[index];
            m_pair_results[index] = find_intersecting_islands(island_entityA, aabb_view, multi_resident_view, tree_view_view);
        });

        auto &manifold_map = m_registry->ctx<contact_manifold_map>();

        for (auto &results : m_pair_results) {
            for (auto &pair : results) {
                if (!manifold_map.contains(pair)) {
                    make_contact_manifold(*m_registry, pair.first, pair.second, m_separation_threshold);
                }
            }
        }

        m_pair_results.clear();
    } else {
        for (auto island_entityA : awake_island_entities) {
            auto pairs = find_intersecting_islands(island_entityA, aabb_view, multi_resident_view, tree_view_view);

            for (auto &pair : pairs) {
                make_contact_manifold(*m_registry, pair.first, pair.second, m_separation_threshold);
            }
        }
    }
}

entity_pair_vector broadphase_main::find_intersecting_islands(entt::entity island_entityA,
                                                              const aabb_view_t &aabb_view,
                                                              const multi_resident_view_t &resident_view,
                                                              const tree_view_view_t &tree_view_view) const {
    auto tree_viewA = tree_view_view.get<tree_view>(island_entityA);
    auto island_aabb = tree_viewA.root_aabb().inset(m_aabb_offset);
    entity_pair_vector results;

    // Query the dynamic tree to find other islands whose AABB intersects the
    // current island's AABB.
    m_island_tree.query(island_aabb, [&] (tree_node_id_t idB) {
        auto island_entityB = m_island_tree.get_node(idB).entity;

        if (island_entityA == island_entityB) {
            return;
        }

        // Look for AABB intersections between entities from different islands
        // and create manifolds.
        auto &tree_viewB = tree_view_view.get<tree_view>(island_entityB);
        auto pairs = intersect_islands(tree_viewA, tree_viewB, aabb_view);
        results.insert(results.end(), pairs.begin(), pairs.end());
    });

    // Query the non-procedural dynamic tree to find static and kinematic
    // entities that are intersecting this island.
    m_np_tree.query(island_aabb, [&] (tree_node_id_t id_np) {
        auto np_entity = m_np_tree.get_node(id_np).entity;

        // Only proceed if the non-procedural entity is not in the island,
        // because if it is already in, collisions are handled in the
        // island worker.
        auto &resident = resident_view.get<multi_island_resident>(np_entity);
        if (resident.island_entities.contains(island_entityA)) {
            return;
        }

        auto pairs = intersect_island_np(tree_viewA, np_entity, aabb_view);
        results.insert(results.end(), pairs.begin(), pairs.end());
    });

    return results;
}

entity_pair_vector broadphase_main::intersect_islands(const tree_view &tree_viewA, const tree_view &tree_viewB,
                                                      const aabb_view_t &aabb_view) const {
    // Query one tree for each node of the other tree. Pick the smaller tree
    // for the iteration and use the bigger one for the query.
    if (tree_viewA.size() < tree_viewB.size()) {
        return intersect_islands_a(tree_viewA, tree_viewB, aabb_view);
    } else {
        return intersect_islands_a(tree_viewB, tree_viewA, aabb_view);
    }
}

entity_pair_vector broadphase_main::intersect_islands_a(const tree_view &tree_viewA, const tree_view &tree_viewB,
                                                        const aabb_view_t &aabb_view) const {
    entity_pair_vector results;
    auto &manifold_map = m_registry->ctx<contact_manifold_map>();

    // `tree_viewA` is iterated and for each node an AABB query is performed in
    // `tree_viewB`, thus for better performance `tree_viewA` should be smaller
    // than `tree_viewB`.
    tree_viewA.each([&] (const tree_view::tree_node &nodeA) {
        auto entityA = nodeA.entity;

        auto aabbA = aabb_view.get<AABB>(entityA).inset(m_aabb_offset);

        tree_viewB.query(aabbA, [&] (tree_node_id_t idB) {
            auto entityB = tree_viewB.get_node(idB).entity;

            if (should_collide(entityA, entityB) && !manifold_map.contains(entityA, entityB)) {
                auto &aabbB = aabb_view.get<AABB>(entityB);

                if (intersect(aabbA, aabbB)) {
                    results.emplace_back(entityA, entityB);
                }
            }
        });
    });

    return results;
}

entity_pair_vector broadphase_main::intersect_island_np(const tree_view &island_tree, entt::entity np_entity,
                                                        const aabb_view_t &aabb_view) const {
    auto np_aabb = aabb_view.get<AABB>(np_entity).inset(m_aabb_offset);
    entity_pair_vector results;
    auto &manifold_map = m_registry->ctx<contact_manifold_map>();

    island_tree.query(np_aabb, [&] (tree_node_id_t idA) {
        auto entity = island_tree.get_node(idA).entity;

        if (should_collide(entity, np_entity) && !manifold_map.contains(entity, np_entity)) {
            auto &aabb = aabb_view.get<AABB>(entity);

            if (intersect(aabb, np_aabb)) {
                results.emplace_back(entity, np_entity);
            }
        }
    });

    return results;
}

bool broadphase_main::should_collide(entt::entity first, entt::entity second) const {
    // Entities should never be equal because they should come from
    // different islands at this point.
    EDYN_ASSERT(first != second);
    auto &settings = m_registry->ctx<edyn::settings>();
    return (*settings.should_collide_func)(*m_registry, first, second);
}

}
