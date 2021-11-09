#include "edyn/networking/update_aabbs_of_interest.hpp"
#include "edyn/networking/aabb_of_interest.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/comp/island.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_aabbs_of_interest(entt::registry &registry) {
    auto &bphase = registry.ctx<broadphase_main>();

    auto view = registry.view<aabb_of_interest>();
    view.each([&] (aabb_of_interest &aabb_of) {
        entt::sparse_set contained_entities;

        bphase.query_islands(aabb_of.aabb, [&] (entt::entity island_entity) {
            auto &nodes = registry.get<edyn::island>(island_entity).nodes;

            for (auto entity : nodes) {
                if (!contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }
        });

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

        aabb_of.entities = std::move(contained_entities);
    });
}

}