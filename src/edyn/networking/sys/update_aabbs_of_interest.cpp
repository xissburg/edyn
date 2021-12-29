#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/comp/island.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_aabbs_of_interest(entt::registry &registry) {
    auto &bphase = registry.ctx<broadphase_main>();

    auto view = registry.view<aabb_of_interest>();
    view.each([&] (aabb_of_interest &aabb_of) {
        entt::sparse_set contained_entities;

        // Collect entities of islands which intersect the AABB of interest.
        bphase.query_islands(aabb_of.aabb, [&] (entt::entity island_entity) {
            auto &island = registry.get<edyn::island>(island_entity);

            for (auto entity : island.nodes) {
                if (!contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }

            for (auto entity : island.edges) {
                if (!contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }
        });

        // Calculate which entities have entered and exited the AABB of interest.
        for (auto entity : aabb_of.entities) {
            if (!contained_entities.contains(entity)) {
                aabb_of.destroy_entities.push_back(entity);
            }
        }

        for (auto entity : contained_entities) {
            if (!aabb_of.entities.contains(entity)) {
                aabb_of.create_entities.push_back(entity);
            }
        }

        // Assign the current set of entities which are in an island that
        // intersects the AABB of interest.
        aabb_of.entities = std::move(contained_entities);
    });
}

}