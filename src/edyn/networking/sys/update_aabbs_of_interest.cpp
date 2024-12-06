#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/core/entity_graph.hpp"
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
    auto &bphase = registry.ctx().get<broadphase>();

    registry.view<aabb_of_interest>().each([&](aabb_of_interest &aabboi) {
        entt::sparse_set contained_entities;

        // Collect entities of islands which intersect the AABB of interest.
        bphase.query_islands(aabboi.aabb, [&](entt::entity island_entity) {
            auto &island = registry.get<edyn::island>(island_entity);

            for (auto entity : island.nodes) {
                if (networked_view.contains(entity) && !contained_entities.contains(entity)) {
                    contained_entities.push(entity);
                }
            }

            for (auto entity : island.edges) {
                if (networked_view.contains(entity) && !contained_entities.contains(entity)) {
                    contained_entities.push(entity);
                }
            }
        });

        bphase.query_non_procedural(aabboi.aabb, [&](entt::entity np_entity) {
            if (networked_view.contains(np_entity) && !contained_entities.contains(np_entity)) {
                contained_entities.push(np_entity);
            }
        });

        // Calculate which entities have entered and exited the AABB of interest.
        for (auto entity : aabboi.entities) {
            if (!contained_entities.contains(entity)) {
                aabboi.entities_exited.push_back(entity);
            }
        }

        for (auto entity : contained_entities) {
            if (!aabboi.entities.contains(entity)) {
                aabboi.entities_entered.push_back(entity);
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
    auto &ctx = registry.ctx().get<aabb_of_interest_async_context>();
    auto query_entity = ctx.id_entity_map.at(id);
    ctx.id_entity_map.erase(id);

    if (!registry.valid(query_entity)) {
        return;
    }

    auto &graph = registry.ctx().get<entity_graph>();
    auto node_view = registry.view<graph_node>();
    auto edge_view = registry.view<graph_edge>();
    auto manifold_view = registry.view<contact_manifold>();
    auto networked_view = registry.view<networked_tag>();
    auto contained_entities = entt::sparse_set{};
    auto to_visit = entt::sparse_set{};

    for (auto entity : result.procedural_entities) {
        if (edge_view.contains(entity)) {
            auto [edge] = edge_view.get(entity);
            auto edge_node_entity = graph.edge_node_entities(edge.edge_index).first;

            if (!to_visit.contains(edge_node_entity)) {
                to_visit.push(edge_node_entity);
            }
        } else if (!to_visit.contains(entity)) {
            EDYN_ASSERT(node_view.contains(entity));
            to_visit.push(entity);
        }
    }

    while (!to_visit.empty()) {
        auto entity = *to_visit.begin();
        auto [node] = node_view.get(entity);

        graph.traverse(node.node_index, [&](auto node_index) {
            auto node_entity = graph.node_entity(node_index);
            to_visit.remove(node_entity);

            if (!contained_entities.contains(node_entity) && networked_view.contains(node_entity)) {
                contained_entities.push(node_entity);
            }
        }, [&](auto edge_index) {
            auto edge_entity = graph.edge_entity(edge_index);

            if (!contained_entities.contains(edge_entity) && networked_view.contains(edge_entity) && !manifold_view.contains(edge_entity)) {
                contained_entities.push(edge_entity);
            }
        });
    }

    for (auto entity : result.non_procedural_entities) {
        if (!contained_entities.contains(entity) && networked_view.contains(entity)) {
            contained_entities.push(entity);
        }
    }

    auto &aabboi = registry.get<aabb_of_interest>(query_entity);

    // Calculate which entities have entered and exited the AABB of interest.
    for (auto entity : aabboi.entities) {
        if (!contained_entities.contains(entity)) {
            aabboi.entities_exited.push_back(entity);
        }
    }

    for (auto entity : contained_entities) {
        if (!aabboi.entities.contains(entity)) {
            aabboi.entities_entered.push_back(entity);
        }
    }

    // Assign the current set of entities which are in an island that
    // intersects the AABB of interest.
    aabboi.entities = std::move(contained_entities);
}

void update_aabbs_of_interest(entt::registry &registry) {
    follow_aabb_of_interest(registry);

    auto &settings = registry.ctx().get<edyn::settings>();
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

        auto &ctx = registry.ctx().get<aabb_of_interest_async_context>();
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
