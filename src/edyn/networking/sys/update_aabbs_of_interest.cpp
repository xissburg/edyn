#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/networking/comp/aabb_oi_follow.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/collision/query_aabb.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <entt/signal/delegate.hpp>
#include <unordered_map>

namespace edyn {

void follow_aabb_of_interest(entt::registry &registry) {
    auto position_view = registry.view<position>();

    registry.view<aabb_of_interest, aabb_oi_follow>().each([&](aabb_of_interest &aabboi, aabb_oi_follow &follow) {
        auto [pos] = position_view.get(follow.entity);
        auto half_size = (aabboi.aabb.max - aabboi.aabb.min) / 2;
        aabboi.aabb.min = pos - half_size;
        aabboi.aabb.max = pos + half_size;
    });
}

void update_aabbs_of_interest_seq(entt::registry &registry) {
    auto networked_view = registry.view<networked_tag>();
    auto &bphase = registry.ctx().at<broadphase>();

    registry.view<aabb_of_interest>().each([&](aabb_of_interest &aabboi) {
        entt::sparse_set contained_entities;
        aabboi.island_entities.clear();

        // Collect entities of islands which intersect the AABB of interest.
        bphase.query_islands(aabboi.aabb, [&](entt::entity island_entity) {
            auto &island = registry.get<edyn::island>(island_entity);

            for (auto entity : island.nodes) {
                if (networked_view.contains(entity) && !contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }

            for (auto entity : island.edges) {
                if (networked_view.contains(entity) && !contained_entities.contains(entity)) {
                    contained_entities.emplace(entity);
                }
            }

            if (!aabboi.island_entities.contains(island_entity)) {
                aabboi.island_entities.emplace(island_entity);
            }
        });

        bphase.query_non_procedural(aabboi.aabb, [&](entt::entity np_entity) {
            if (networked_view.contains(np_entity) && !contained_entities.contains(np_entity)) {
                contained_entities.emplace(np_entity);
            }
        });

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

struct aabb_of_interest_async_context {
    std::unordered_map<query_aabb_id_type, entt::entity> id_entity_map;
};

void process_aabb_of_interest_result(entt::registry &registry, query_aabb_id_type id,
                                     const query_aabb_result &result) {
    auto &ctx = registry.ctx().at<aabb_of_interest_async_context>();
    auto entity = ctx.id_entity_map.at(id);
    ctx.id_entity_map.erase(id);

    if (!registry.valid(entity)) {
        return;
    }

    auto &aabboi = registry.get<aabb_of_interest>(entity);

    // Insert island entities.
    aabboi.island_entities.clear();

    for (auto island_entity : result.island_entities) {
        if (!aabboi.island_entities.contains(island_entity)) {
            aabboi.island_entities.emplace(island_entity);
        }
    }

    // Insert owners of each entity.
    entt::sparse_set client_entities;
    auto owner_view = registry.view<entity_owner>();

    for (auto entity : result.procedural_entities) {
        if (owner_view.contains(entity)) {
            auto client_entity = owner_view.get<entity_owner>(entity).client_entity;

            if (!client_entities.contains(client_entity)) {
                client_entities.emplace(client_entity);
            }
        }
    }

    auto contained_entities = entt::sparse_set{};
    contained_entities.insert(result.procedural_entities.begin(), result.procedural_entities.end());
    contained_entities.insert(result.non_procedural_entities.begin(), result.non_procedural_entities.end());

    contained_entities.insert(client_entities.begin(), client_entities.end());
    auto networked_view = registry.view<networked_tag>();

    // Calculate which entities have entered and exited the AABB of interest.
    for (auto entity : aabboi.entities) {
        if (networked_view.contains(entity) && !contained_entities.contains(entity)) {
            aabboi.destroy_entities.push_back(entity);
        }
    }

    for (auto entity : contained_entities) {
        if (networked_view.contains(entity) && !aabboi.entities.contains(entity)) {
            aabboi.create_entities.push_back(entity);
        }
    }

    // Assign the current set of entities which are in an island that
    // intersects the AABB of interest.
    aabboi.entities = std::move(contained_entities);
}

void update_aabbs_of_interest(entt::registry &registry) {
    follow_aabb_of_interest(registry);

    auto &settings = registry.ctx().at<edyn::settings>();
    auto exec_mode = settings.execution_mode;

    switch (exec_mode) {
    case execution_mode::sequential:
    case execution_mode::sequential_multithreaded:
        update_aabbs_of_interest_seq(registry);
        break;

    case execution_mode::asynchronous: {
        if (!registry.ctx().contains<aabb_of_interest_async_context>()) {
            registry.ctx().emplace<aabb_of_interest_async_context>();
        }

        auto &ctx = registry.ctx().at<aabb_of_interest_async_context>();
        auto delegate = entt::delegate(entt::connect_arg_t<&process_aabb_of_interest_result>{}, registry);

        for (auto [entity, aabboi] : registry.view<aabb_of_interest>().each()) {
            auto id = query_aabb_of_interest_async(registry, aabboi.aabb, delegate);
            ctx.id_entity_map[id] = entity;
        }
        break;
    }
    }
}

}
