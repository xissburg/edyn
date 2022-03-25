#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/networking/comp/aabb_oi_follow.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_aabbs_of_interest(entt::registry &registry) {
    auto &bphase = registry.ctx<broadphase_main>();
    auto owner_view = registry.view<entity_owner>();
    auto manifold_view = registry.view<contact_manifold>();
    auto position_view = registry.view<position>();

    registry.view<aabb_of_interest, aabb_oi_follow>().each([&] (aabb_of_interest &aabboi, aabb_oi_follow &follow) {
        auto [pos] = position_view.get(follow.entity);
        auto half_size = (aabboi.aabb.max - aabboi.aabb.min) / 2;
        aabboi.aabb.min = pos - half_size;
        aabboi.aabb.max = pos + half_size;
    });

    registry.view<aabb_of_interest>().each([&] (aabb_of_interest &aabboi) {
        entt::sparse_set contained_entities;

        aabboi.island_entities.clear();
        // Collect entities of islands which intersect the AABB of interest.
        bphase.query_islands(aabboi.aabb, [&] (entt::entity island_entity) {
            auto &island = registry.get<edyn::island>(island_entity);

            for (auto entity : island.nodes) {
                if (!contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }

            for (auto entity : island.edges) {
                // Ignore contact manifolds.
                if (manifold_view.contains(entity)) {
                    continue;
                }

                if (!contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }

            if (!aabboi.island_entities.contains(island_entity)) {
                aabboi.island_entities.emplace(island_entity);
            }
        });

        bphase.query_non_procedural(aabboi.aabb, [&] (entt::entity np_entity) {
            if (!contained_entities.contains(np_entity)) {
                contained_entities.emplace(np_entity);
            }
        });

        // Insert owners of each entity.
        entt::sparse_set client_entities;

        for (auto entity : contained_entities) {
            if (owner_view.contains(entity)) {
                auto client_entity = owner_view.get<entity_owner>(entity).client_entity;

                if (!client_entities.contains(client_entity)) {
                    client_entities.emplace(client_entity);
                }
            }
        }

        contained_entities.insert(client_entities.begin(), client_entities.end());

        // Calculate which entities have entered and exited the AABB of interest.
        for (auto entity : aabboi.entities) {
            if (!contained_entities.contains(entity)) {
                aabboi.destroy_entities.push_back(entity);
            }
        }

        for (auto entity : contained_entities) {
            if (!aabboi.entities.contains(entity)) {
                aabboi.create_entities.push_back(entity);
            }
        }

        // Assign the current set of entities which are in an island that
        // intersects the AABB of interest.
        aabboi.entities = std::move(contained_entities);
    });
}

}
