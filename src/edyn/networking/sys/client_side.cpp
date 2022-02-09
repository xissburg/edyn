#include "edyn/networking/sys/client_side.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/edyn.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/packet/general_snapshot.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/networking/util/non_proc_comp_state_history.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/tuple_util.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

void on_construct_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.importing_entities) {
        ctx.created_entities.push_back(entity);
    }
}

void on_destroy_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.importing_entities) {
        ctx.destroyed_entities.push_back(entity);

        if (ctx.entity_map.has_loc(entity)) {
            ctx.entity_map.erase_loc(entity);
        }
    }
}

void on_construct_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.emplace(entity);
    }
}

void on_destroy_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.erase(entity);
    }
}

static void update_non_proc_comp_state_history(entt::registry &registry,
                                               non_proc_comp_state_history &state_history,
                                               double timestamp) {
    // Insert select non-procedural components into history only for entities
    // owned by the local client.
    auto &settings = registry.ctx<edyn::settings>();
    auto builder = (*settings.make_island_delta_builder)();
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto entity : ctx.owned_entities) {
        ctx.pool_snapshot_importer->insert_non_procedural_to_builder(registry, entity, *builder);
    }

    if (!builder->empty()) {
        state_history.emplace(builder->finish(), timestamp);
    }
}

void init_networking_client(entt::registry &registry) {
    registry.set<client_networking_context>();

    registry.on_construct<networked_tag>().connect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().connect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().connect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().connect<&on_destroy_entity_owner>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.networking_settings = client_networking_settings{};
}

void deinit_networking_client(entt::registry &registry) {
    registry.unset<client_networking_context>();

    registry.on_construct<networked_tag>().disconnect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().disconnect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().disconnect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().disconnect<&on_destroy_entity_owner>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.networking_settings = {};
}

static void apply_extrapolation_result(entt::registry &registry, extrapolation_result &result) {
    entt::sparse_set island_entities;

    for (auto entity : result.entities) {
        // Entity could've been destroyed while extrapolation was running.
        if (!registry.valid(entity)) {
            continue;
        }

        if (auto *resident = registry.try_get<island_resident>(entity)) {
            if (!island_entities.contains(resident->island_entity)) {
                island_entities.emplace(resident->island_entity);
            }
        } else if (auto *resident = registry.try_get<multi_island_resident>(entity)) {
            for (auto island_entity : resident->island_entities) {
                if (!island_entities.contains(island_entity)) {
                    island_entities.emplace(island_entity);
                }
            }
        }
    }

    EDYN_ASSERT(!island_entities.empty());
    auto &coordinator = registry.ctx<island_coordinator>();

    for (auto island_entity : island_entities) {
        coordinator.send_island_message<extrapolation_result>(island_entity, result);
        coordinator.wake_up_island(island_entity);
    }
}

void update_networking_client(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.created_entities.empty()) {
        packet::create_entity packet;
        packet.entities = ctx.created_entities;

        for (auto entity : ctx.created_entities) {
            ctx.pool_snapshot_exporter->export_all(registry, entity, packet.pools);
            registry.emplace<entity_owner>(entity, ctx.client_entity);
        }

        std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        ctx.created_entities.clear();
    }

    if (!ctx.destroyed_entities.empty()) {
        packet::destroy_entity packet;
        packet.entities = std::move(ctx.destroyed_entities);
        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
    }

    auto time = performance_time();

    if (time - ctx.last_snapshot_time > 1 / ctx.snapshot_rate) {
        auto packet = packet::transient_snapshot{};

        for (auto entity : ctx.owned_entities) {
            EDYN_ASSERT(registry.all_of<networked_tag>(entity));
            ctx.pool_snapshot_exporter->export_transient(registry, entity, packet.pools);
        }

        ctx.last_snapshot_time = time;

        if (!packet.pools.empty()) {
            ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        }
    }

    // Check if extrapolation jobs are finished and merge their results into
    // the main registry.
    auto remove_it = std::remove_if(ctx.extrapolation_jobs.begin(), ctx.extrapolation_jobs.end(),
                                    [&] (extrapolation_job_context &extr_ctx) {
        if (extr_ctx.job->is_finished()) {
            auto &result = extr_ctx.job->get_result();
            apply_extrapolation_result(registry, result);
            return true;
        }
        return false;
    });
    ctx.extrapolation_jobs.erase(remove_it, ctx.extrapolation_jobs.end());

    update_non_proc_comp_state_history(registry, ctx.state_history, time);

    auto dirty_view = registry.view<dirty, networked_tag>();

    if (dirty_view.size_hint() > 0) {
        auto packet = packet::general_snapshot{};

        for (auto [entity, dirty] : dirty_view.each()) {
            for (auto id : dirty.updated_indexes) {
                ctx.pool_snapshot_exporter->export_by_type_id(registry, entity, id, packet.pools);
            }
        }

        if (!packet.pools.empty()) {
            ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        }
    }
}

static void process_packet(entt::registry &registry, const packet::client_created &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto remote_entity = packet.client_entity;
    auto local_entity = registry.create();
    edyn::tag_external_entity(registry, local_entity, false);

    EDYN_ASSERT(ctx.client_entity == entt::null);
    ctx.client_entity = local_entity;
    ctx.client_entity_assigned_signal.publish();
    ctx.entity_map.insert(remote_entity, local_entity);

    auto emap_packet = packet::update_entity_map{};
    emap_packet.pairs.emplace_back(remote_entity, local_entity);
    ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});

    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::update_entity_map &emap) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pair : emap.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;
        ctx.entity_map.insert(remote_entity, local_entity);
    }
}

static void process_packet(entt::registry &registry, const packet::entity_request &req) {

}

static void process_packet(entt::registry &registry, const packet::entity_response &res) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto emap_packet = packet::update_entity_map{};

    for (auto remote_entity : res.entities) {
        if (ctx.entity_map.has_rem(remote_entity)) {
            continue;
        }

        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    for (auto &pool : res.pools) {
        ctx.pool_snapshot_importer->import(registry, ctx.entity_map, pool);
    }

    for (auto remote_entity : res.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);
        registry.emplace<networked_tag>(local_entity);
    }

    ctx.importing_entities = false;

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
    }
}

template<typename T>
void create_graph_edge(entt::registry &registry, entt::entity entity) {
    if (registry.any_of<graph_edge>(entity)) return;

    auto &comp = registry.get<T>(entity);
    auto node_index0 = registry.get<graph_node>(comp.body[0]).node_index;
    auto node_index1 = registry.get<graph_node>(comp.body[1]).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
    registry.emplace<graph_edge>(entity, edge_index);
}

template<typename... Ts>
void maybe_create_graph_edge(entt::registry &registry, entt::entity entity, [[maybe_unused]] std::tuple<Ts...>) {
    ((registry.any_of<Ts>(entity) ? create_graph_edge<Ts>(registry, entity) : void(0)), ...);
}

static void process_packet(entt::registry &registry, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    // Collect new entity mappings to send back to server.
    auto emap_packet = packet::update_entity_map{};

    // Create entities first...
    for (auto remote_entity : packet.entities) {
        if (ctx.entity_map.has_rem(remote_entity)) continue;

        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
    }

    // ... assign components later so that entity references will be available
    // to be mapped into the local registry.
    for (auto &pool : packet.pools) {
        ctx.pool_snapshot_importer->import(registry, ctx.entity_map, pool);
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (!registry.all_of<networked_tag>(local_entity)) {
            registry.emplace<networked_tag>(local_entity);
        }
    }

    // Create nodes and edges in entity graph.
    for (auto remote_entity : packet.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }

        if (registry.any_of<procedural_tag>(local_entity)) {
            registry.emplace<discontinuity>(local_entity);
        }
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);
        maybe_create_graph_edge(registry, local_entity, constraints_tuple);
    }

    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::destroy_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    for (auto remote_entity : packet.entities) {
        if (!ctx.entity_map.has_rem(remote_entity)) continue;

        auto local_entity = ctx.entity_map.remloc(remote_entity);
        ctx.entity_map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }

    ctx.importing_entities = false;
}

static void collect_unknown_entities(const entt::registry &registry, entity_map &entity_map,
                                     const std::vector<entt::entity> &remote_entities,
                                     entt::sparse_set &unknown_entities) {
    // Find remote entities that have no local counterpart.
    for (auto remote_entity : remote_entities) {
        if (entity_map.has_rem(remote_entity)) {
            auto local_entity = entity_map.remloc(remote_entity);

            // In the unusual situation where an existing mapping is an invalid
            // entity, erase it from the entity map and consider it unknown.
            if (!registry.valid(local_entity)) {
                entity_map.erase_loc(local_entity);

                if (!unknown_entities.contains(remote_entity)) {
                    unknown_entities.emplace(remote_entity);
                }
            }
        } else if (!unknown_entities.contains(remote_entity)) {
            unknown_entities.emplace(remote_entity);
        }
    }
}

static void request_unknown_entities_in_pools(entt::registry &registry,
                                              const std::vector<pool_snapshot> &pools) {
    auto &ctx = registry.ctx<client_networking_context>();
    entt::sparse_set unknown_entities;

    for (auto &pool : pools) {
        collect_unknown_entities(registry, ctx.entity_map, pool.ptr->get_entities(), unknown_entities);
    }

    if (!unknown_entities.empty()) {
        // Request unknown entities.
        auto req = packet::entity_request{};
        req.entities.insert(req.entities.end(), unknown_entities.begin(), unknown_entities.end());
        ctx.packet_signal.publish(packet::edyn_packet{std::move(req)});
    }
}

static void process_packet(entt::registry &registry, const packet::transient_snapshot &snapshot) {
    request_unknown_entities_in_pools(registry, snapshot.pools);

    auto &ctx = registry.ctx<client_networking_context>();

    // If extrapolation is not enabled, just import the snapshot immediately.
    if (!ctx.extrapolation_enabled) {
        for (auto &pool : snapshot.pools) {
            ctx.pool_snapshot_importer->import(registry, ctx.entity_map, pool);
        }

        return;
    }

    const auto time = performance_time();
    const double snapshot_time = time - (ctx.server_playout_delay + ctx.round_trip_time / 2);

    // Non-procedural component state from other clients must be added to the
    // input history beforehand.
    auto &settings = registry.ctx<edyn::settings>();

    {
        auto builder = (*settings.make_island_delta_builder)();
        ctx.pool_snapshot_importer->insert_remote_non_procedural_to_builder(registry, snapshot.pools, ctx.entity_map, *builder);

        if (!builder->empty()) {
            ctx.state_history.emplace(builder->finish(), snapshot_time);
        }
    }

    // Ignore it if the number of current extrapolation jobs is at maximum.
    if (ctx.extrapolation_jobs.size() >= ctx.max_concurrent_extrapolations) {
        return;
    }

    // Collect unique entities in the snapshot.
    entt::sparse_set snapshot_entities;

    for (auto &pool : snapshot.pools) {
        auto pool_entities = pool.ptr->get_entities();

        for (auto entity : pool_entities) {
            if (!snapshot_entities.contains(entity)) {
                snapshot_entities.emplace(entity);
            }
        }
    }

    // Collect the node indices of all entities involved in the snapshot.
    std::set<entity_graph::index_type> node_indices;
    auto node_view = registry.view<graph_node>();
    auto &graph = registry.ctx<entity_graph>();

    for (auto remote_entity : snapshot_entities) {
        if (!ctx.entity_map.has_rem(remote_entity)) {
            // Entity not present in client. Send an entity request to server.
            ctx.request_entity_signal.publish(remote_entity);
            continue;
        }

        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (node_view.contains(local_entity)) {
            auto node_index = node_view.get<graph_node>(local_entity).node_index;

            if (graph.is_connecting_node(node_index)) {
                node_indices.insert(node_index);
            }
        }
    }

    if (node_indices.empty()) {
        return;
    }

    // Traverse entity graph to find all entities connected to the entities
    // present in the transient snapshot.
    auto entities = entt::sparse_set{};
    graph.reach(
        node_indices.begin(), node_indices.end(),
        [&] (entt::entity entity) { // visitNodeFunc
            if (!entities.contains(entity)) {
                entities.emplace(entity);
            }
        },
        [&] (entt::entity entity) { // visitEdgeFunc
            if (!entities.contains(entity)) {
                entities.emplace(entity);
            }
        },
        [&] (entity_graph::index_type) { // shouldVisitFunc
            return true;
        },
        [] () { // connectedComponentFunc
        });

    // TODO: only include the necessary static entities.
    for (auto entity : registry.view<static_tag>()) {
        if (!entities.contains(entity)) {
            entities.emplace(entity);
        }
    }

    // Create registry snapshot to send to extrapolation job.
    extrapolation_input input;
    input.extrapolation_component_pool_import_by_id_func = ctx.extrapolation_component_pool_import_by_id_func;
    input.is_non_procedural_component_func = ctx.is_non_procedural_component_func;
    (*ctx.extrapolation_component_pool_import_func)(input.pools, registry, entities);
    input.start_time = snapshot_time;

    for (auto entity : entities) {
        if (auto *owner = registry.try_get<entity_owner>(entity);
            owner && owner->client_entity == ctx.client_entity)
        {
            input.owned_entities.emplace(entity);
        }
    }

    input.entities = std::move(entities);

    // Translate transient snapshot into client's space before sending it to
    // the extrapolation job, or else it won't be able to assimilate the
    // server-side entities with client side-entities.
    input.transient_snapshot = snapshot;
    input.transient_snapshot.convert_remloc(ctx.entity_map);

    // Create extrapolation job and put the registry snapshot and the transient
    // snapshot into its message queue.
    auto &material_table = registry.ctx<material_mix_table>();

    auto job = std::make_unique<extrapolation_job>(std::move(input), settings, material_table, ctx.state_history);
    job->reschedule();

    ctx.extrapolation_jobs.push_back(extrapolation_job_context{std::move(job)});
}

static void process_packet(entt::registry &registry, const packet::general_snapshot &snapshot) {
    request_unknown_entities_in_pools(registry, snapshot.pools);

    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pool : snapshot.pools) {
        ctx.pool_snapshot_importer->import(registry, ctx.entity_map, pool);
    }
}

static void process_packet(entt::registry &registry, const packet::set_playout_delay &delay) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.server_playout_delay = delay.value;
}

void client_handle_packet(entt::registry &registry, const packet::edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

entt::sink<void()> on_client_entity_assigned(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();
    return entt::sink{ctx.client_entity_assigned_signal};
}

bool client_owns_entity(const entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    return ctx.client_entity == registry.get<entity_owner>(entity).client_entity;
}

}
