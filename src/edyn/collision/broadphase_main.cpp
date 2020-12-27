#include "edyn/collision/broadphase_main.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/parallel/island_worker_context.hpp"
#include <entt/entt.hpp>
#include <vector>

namespace edyn {

broadphase_main::broadphase_main(entt::registry &registry)
    : m_registry(&registry)
    , m_manifold_map(registry)
{}

void broadphase_main::update() {
    // A higher threshold is used in the main broadphase to create contact 
    // manifolds between different islands a little earlier and decrease the
    // probability they'll arrive in the corresponding island worker when the
    // AABBs are already intersecting.
    const auto threshold = contact_breaking_threshold * 4;
    const auto offset = vector3_one * -threshold;
    const auto separation_threshold = threshold * 1.3;

    // Update AABBs in tree.
    auto tree_view_node_view = m_registry->view<tree_view, tree_node_id_t>();
    tree_view_node_view.each([&] (entt::entity, tree_view &tree_view, tree_node_id_t node_id) {
        m_tree.move(node_id, tree_view.root_aabb());
    });

    // Search for island pairs with intersecting AABBs, i.e. the AABB of the root
    // node of their trees intersect.
    m_registry->view<tree_view>().each([&] (entt::entity island_entityA, tree_view &tree_viewA) {
        auto island_aabb = tree_viewA.root_aabb().inset(offset);
        
        m_tree.query(island_aabb, [&] (tree_node_id_t id) {
            auto island_entityB = m_tree.get_node(id).entity;

            // Look for AABB intersections between entities from different islands
            // and create manifolds.
            auto aabb_view = m_registry->view<AABB>();
            auto &ctxB = m_registry->get<island_worker_context>(island_entityB);

            for (auto entity : ctxB.m_entities) {
                auto offset_aabb = aabb_view.get(entity).inset(offset);
                tree_viewA.query(offset_aabb, [&] (tree_node_id_t id) {
                    auto &node = m_tree.get_node(id);
                    auto p = std::make_pair(entity, node.entity);

                    if (!m_manifold_map.contains(p) && should_collide(entity, node.entity)) {
                        auto &other_aabb = aabb_view.get(node.entity);

                        if (intersect(offset_aabb, other_aabb)) {
                            make_contact_manifold(*m_registry, entity, node.entity, separation_threshold);
                        }
                    }

                    return true;
                });
            }

            return true;
        });
    });
}

bool broadphase_main::should_collide(entt::entity e0, entt::entity e1) const {
    if (e0 == e1) {
        return false;
    }

    // Only consider nodes that reside in different islands.
    auto &container0 = m_registry->get<island_container>(e0);
    auto &container1 = m_registry->get<island_container>(e1);

    // One of the entities must be dynamic.
    if (m_registry->has<dynamic_tag>(e0)) {
        // Dynamic entities are assigned to only one island.
        auto island_entity = *container0.entities.begin();
        for (auto other_island_entity : container1.entities) {
            if (other_island_entity == island_entity) {
                return false;
            }
        }
    } else if (m_registry->has<dynamic_tag>(e1)) {
        auto island_entity = *container1.entities.begin();
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