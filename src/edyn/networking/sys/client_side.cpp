#include "edyn/networking/sys/client_side.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/edyn.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/util/process_update_entity_map_packet.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/packet/general_snapshot.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/vector.hpp"
#include "edyn/util/aabb_util.hpp"
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

void on_construct_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_network_context>();

    if (!ctx.importing_entities) {
        ctx.created_entities.push_back(entity);
    }
}

void on_destroy_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_network_context>();

    if (!ctx.importing_entities) {
        ctx.destroyed_entities.push_back(entity);
    }
}

void on_construct_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_network_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.emplace(entity);
    }
}

void on_destroy_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_network_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.erase(entity);
    }
}

static void update_input_history(entt::registry &registry, double timestamp) {
    // Insert input components into history only for entities owned by the
    // local client.
    auto &ctx = registry.ctx<client_network_context>();
    ctx.state_history->emplace(registry, ctx.owned_entities, timestamp);

    // Erase all inputs until the current time minus the client-server time
    // difference plus some leeway because this is the amount of time the
    // transient snapshots will be extrapolated forward thus requiring the
    // inputs from that point in time onwards.
    auto &settings = registry.ctx<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    const auto client_server_time_difference = ctx.server_playout_delay + client_settings.round_trip_time / 2;
    ctx.state_history->erase_until(timestamp - (client_server_time_difference * 1.1 + 0.2));
}

void init_network_client(entt::registry &registry) {
    registry.set<client_network_context>();

    registry.on_construct<networked_tag>().connect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().connect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().connect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().connect<&on_destroy_entity_owner>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.network_settings = client_network_settings{};
}

void deinit_network_client(entt::registry &registry) {
    registry.unset<client_network_context>();

    registry.on_construct<networked_tag>().disconnect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().disconnect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().disconnect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().disconnect<&on_destroy_entity_owner>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.network_settings = {};
}

static void process_created_networked_entities(entt::registry &registry, double time) {
    auto &ctx = registry.ctx<client_network_context>();

    if (ctx.created_entities.empty()) {
        return;
    }

    packet::create_entity packet;
    packet.timestamp = time;
    packet.entities = std::move(ctx.created_entities);

    ctx.snapshot_exporter->export_all(registry, packet);

    for (auto entity : packet.entities) {
        registry.emplace<entity_owner>(entity, ctx.client_entity);
    }

    // Sort components to ensure order of construction.
    std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
        return lhs.component_index < rhs.component_index;
    });

    ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
}

static void process_destroyed_networked_entities(entt::registry &registry, double time) {
    auto &ctx = registry.ctx<client_network_context>();

    if (ctx.destroyed_entities.empty()) {
        return;
    }

    packet::destroy_entity packet;
    packet.timestamp = time;
    packet.entities = std::move(ctx.destroyed_entities);
    ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
}

static void maybe_publish_transient_snapshot(entt::registry &registry, double time) {
    auto &ctx = registry.ctx<client_network_context>();
    auto &settings = registry.ctx<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    if (time - ctx.last_snapshot_time < 1 / client_settings.snapshot_rate) {
        return;
    }

    ctx.last_snapshot_time = time;

    auto packet = packet::transient_snapshot{};
    packet.timestamp = time;

    if (ctx.allow_full_ownership) {
        // Include transient components of all entities in the islands that
        // contain an entity owned by this client, excluding entities that are
        // owned by other clients.
        auto island_entities = collect_islands_from_residents(registry, ctx.owned_entities.begin(), ctx.owned_entities.end());
        auto island_view = registry.view<island>();
        auto networked_view = registry.view<networked_tag>();
        auto owner_view = registry.view<entity_owner>();

        auto should_include_entity = [&] (entt::entity entity) {
            if (!networked_view.contains(entity)) {
                return false;
            }

            auto is_owned_by_another_client =
                owner_view.contains(entity) &&
                std::get<0>(owner_view.get(entity)).client_entity != ctx.client_entity;

            return !is_owned_by_another_client &&
                   ctx.snapshot_exporter->contains_transient(registry, entity);
        };

        for (auto island_entity : island_entities) {
            auto [island] = island_view.get(island_entity);

            for (auto entity : island.nodes) {
                if (should_include_entity(entity)) {
                    packet.entities.push_back(entity);
                }
            }

            for (auto entity : island.edges) {
                if (should_include_entity(entity)) {
                    packet.entities.push_back(entity);
                }
            }
        }

        ctx.snapshot_exporter->export_transient(registry, packet);
    } else {
        // Only include transient input components of entities owned by this client.
        for (auto entity : ctx.owned_entities) {
            if (ctx.snapshot_exporter->contains_transient_input(registry, entity)) {
                packet.entities.push_back(entity);
            }
        }

        ctx.snapshot_exporter->export_transient_input(registry, packet);
    }

    if (!packet.entities.empty() && !packet.pools.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
    }
}

static void apply_extrapolation_result(entt::registry &registry, extrapolation_result &result) {
    // Result contains entities already mapped into the main registry space.
    // Entities could've been destroyed while extrapolation was running.
    auto invalid_it = std::remove_if(result.entities.begin(), result.entities.end(),
                                     [&] (auto entity) { return !registry.valid(entity); });
    result.entities.erase(invalid_it, result.entities.end());

    auto island_entities = collect_islands_from_residents(registry, result.entities.begin(), result.entities.end());
    auto &coordinator = registry.ctx<island_coordinator>();

    for (auto island_entity : island_entities) {
        coordinator.send_island_message<extrapolation_result>(island_entity, result);
        coordinator.wake_up_island(island_entity);
    }

    if (result.terminated_early) {
        auto &ctx = registry.ctx<client_network_context>();
        ctx.extrapolation_timeout_signal.publish();
    }
}

static void process_finished_extrapolation_jobs(entt::registry &registry) {
    auto &ctx = registry.ctx<client_network_context>();

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
}

static void publish_dirty_components(entt::registry &registry, double time) {
    // Share dirty networked entities using a general snapshot.
    auto dirty_view = registry.view<dirty, networked_tag>();

    if (dirty_view.size_hint() == 0) {
        return;
    }

    auto &ctx = registry.ctx<client_network_context>();
    auto packet = packet::general_snapshot{};
    packet.timestamp = time;

    // Collect all entities first. Transient components that have been marked
    // as dirty must be ignored, since they're updated regularly via the
    // transient snapshots.
    for (auto [entity, dirty] : dirty_view.each()) {
        for (auto id : dirty.updated_indexes) {
            if (!ctx.snapshot_exporter->is_transient(id)) {
                packet.entities.push_back(entity);
                break;
            }
        }
    }

    // Add components later because all entities involved must be known at this
    // point so entity indices can be assigned in the registry snapshot of the
    // packet.
    for (auto [entity, dirty] : dirty_view.each()) {
        for (auto id : dirty.updated_indexes) {
            if (!ctx.snapshot_exporter->is_transient(id)) {
                ctx.snapshot_exporter->export_by_type_id(registry, entity, id, packet);
            }
        }
    }

    if (!packet.entities.empty() && !packet.pools.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
    }
}

static void merge_network_dirty_into_dirty(entt::registry &registry) {
    auto dirty_view = registry.view<dirty>();

    // Insert components marked as dirty during snapshot import into the regular
    // dirty component. This is done separately to avoid having the components
    // marked as dirty during snapshot import being sent back to the server
    // in `publish_dirty_components`.
    for (auto [entity, network_dirty] : registry.view<network_dirty>().each()) {
        if (!dirty_view.contains(entity)) {
            registry.emplace<dirty>(entity);
        }

        dirty_view.get<edyn::dirty>(entity).merge(network_dirty);
    }

    registry.clear<network_dirty>();
}

static void client_update_clock_sync(entt::registry &registry, double time) {
    auto &ctx = registry.ctx<client_network_context>();
    auto &settings = registry.ctx<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    update_clock_sync(ctx.clock_sync, time, client_settings.round_trip_time);
}

void update_network_client(entt::registry &registry) {
    auto time = performance_time();

    client_update_clock_sync(registry, time);
    process_created_networked_entities(registry, time);
    process_destroyed_networked_entities(registry, time);
    maybe_publish_transient_snapshot(registry, time);
    process_finished_extrapolation_jobs(registry);
    update_input_history(registry, time);
    publish_dirty_components(registry, time);
    merge_network_dirty_into_dirty(registry);
}

static void process_packet(entt::registry &registry, const packet::client_created &packet) {
    auto &ctx = registry.ctx<client_network_context>();
    ctx.importing_entities = true;

    auto remote_entity = packet.client_entity;
    auto local_entity = registry.create();
    edyn::tag_external_entity(registry, local_entity, false);

    EDYN_ASSERT(ctx.client_entity == entt::null);
    ctx.client_entity = local_entity;
    ctx.entity_map.insert(remote_entity, local_entity);

    auto emap_packet = packet::update_entity_map{};
    emap_packet.timestamp = performance_time();
    emap_packet.pairs.emplace_back(remote_entity, local_entity);
    ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});

    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::update_entity_map &packet) {
    auto &ctx = registry.ctx<client_network_context>();
    process_update_entity_map_packet(registry, packet, ctx.entity_map);
}

static void process_packet(entt::registry &registry, const packet::entity_request &req) {
    // TODO: send response.
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

static void import_remote_snapshot(entt::registry &registry, const registry_snapshot &snap) {
    auto &ctx = registry.ctx<client_network_context>();

    // Collect new entity mappings to send back to server.
    auto emap_packet = packet::update_entity_map{};
    emap_packet.timestamp = performance_time();

    // Create entities first...
    for (auto remote_entity : snap.entities) {
        if (ctx.entity_map.contains(remote_entity)) continue;

        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
    }

    // ... assign components later so that entity references will be available
    // to be mapped into the local registry.

    // Do not mark as dirty during import as it would cause components to be
    // emplaced twice in an island worker since these are newly created
    // entities which will already be fully instantiated with all components
    // in an island worker.
    const bool mark_dirty = false;

    ctx.snapshot_importer->import(registry, ctx.entity_map, snap, mark_dirty);

    // Create nodes and edges in entity graph, assign networked tags and
    // dependent components which are not networked.
    for (auto remote_entity : snap.entities) {
        auto local_entity = ctx.entity_map.at(remote_entity);

        // Assign computed properties such as AABB and inverse mass.
        if (registry.any_of<shape_index>(local_entity)) {
            auto &pos = registry.get<position>(local_entity);
            auto &orn = registry.get<orientation>(local_entity);

            visit_shape(registry, local_entity, [&] (auto &&shape) {
                auto aabb = shape_aabb(shape, pos, orn);
                registry.emplace<AABB>(local_entity, aabb);
            });
        }

        if (auto *mass = registry.try_get<edyn::mass>(local_entity)) {
            EDYN_ASSERT(
                (registry.all_of<dynamic_tag>(local_entity) && *mass > 0 && *mass < EDYN_SCALAR_MAX) ||
                (registry.any_of<kinematic_tag, static_tag>(local_entity) && *mass == EDYN_SCALAR_MAX));
            auto inv = registry.all_of<dynamic_tag>(local_entity) ? scalar(1) / *mass : scalar(0);
            registry.emplace<mass_inv>(local_entity, inv);
        }

        if (auto *inertia = registry.try_get<edyn::inertia>(local_entity)) {
            if (registry.all_of<dynamic_tag>(local_entity)) {
                EDYN_ASSERT(*inertia != matrix3x3_zero);
                auto I_inv = inverse_matrix_symmetric(*inertia);
                registry.emplace<inertia_inv>(local_entity, I_inv);
                registry.emplace<inertia_world_inv>(local_entity, I_inv);
            } else {
                EDYN_ASSERT(*inertia == matrix3x3_zero);
                registry.emplace<inertia_inv>(local_entity, matrix3x3_zero);
                registry.emplace<inertia_world_inv>(local_entity, matrix3x3_zero);
            }
        }

        // Assign discontinuity to dynamic rigid bodies.
        if (registry.any_of<dynamic_tag>(local_entity) && !registry.all_of<discontinuity>(local_entity)) {
            registry.emplace<discontinuity>(local_entity);
        }

        // All remote entities must have a networked tag.
        if (!registry.all_of<networked_tag>(local_entity)) {
            registry.emplace<networked_tag>(local_entity);
        }

        // Assign graph node to rigid bodies and external entities.
        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }
    }

    // Create graph edges for constraints *after* graph nodes have been created
    // for rigid bodies above.
    for (auto remote_entity : snap.entities) {
        auto local_entity = ctx.entity_map.at(remote_entity);
        maybe_create_graph_edge(registry, local_entity, constraints_tuple);
    }
}

static void process_packet(entt::registry &registry, const packet::entity_response &res) {
    auto &ctx = registry.ctx<client_network_context>();
    ctx.importing_entities = true;
    import_remote_snapshot(registry, res);
    ctx.importing_entities = false;

    // Remove received entities from the set of requested entities.
    for (auto remote_entity : res.entities) {
        if (ctx.requested_entities.contains(remote_entity)) {
            ctx.requested_entities.erase(remote_entity);
        }
    }
}

static void process_packet(entt::registry &registry, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<client_network_context>();
    ctx.importing_entities = true;
    import_remote_snapshot(registry, packet);
    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::destroy_entity &packet) {
    auto &ctx = registry.ctx<client_network_context>();
    ctx.importing_entities = true;

    for (auto remote_entity : packet.entities) {
        if (!ctx.entity_map.contains(remote_entity)) continue;

        auto local_entity = ctx.entity_map.at(remote_entity);
        ctx.entity_map.erase(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }

    ctx.importing_entities = false;
}

static void collect_unknown_entities(entt::registry &registry,
                                     const std::vector<entt::entity> &remote_entities,
                                     entt::sparse_set &unknown_entities) {
    auto &ctx = registry.ctx<client_network_context>();

    // Find remote entities that have no local counterpart.
    for (auto remote_entity : remote_entities) {
        if (ctx.entity_map.contains(remote_entity)) {
            auto local_entity = ctx.entity_map.at(remote_entity);

            // In the unusual situation where an existing mapping is an invalid
            // entity, consider it unknown.
            if (!registry.valid(local_entity)) {
                if (!unknown_entities.contains(remote_entity)) {
                    unknown_entities.emplace(remote_entity);
                }
            }
        } else if (!unknown_entities.contains(remote_entity)) {
            unknown_entities.emplace(remote_entity);
        }
    }
}

static bool request_unknown_entities(entt::registry &registry,
                                     const std::vector<entt::entity> &entities) {
    entt::sparse_set unknown_entities;
    collect_unknown_entities(registry, entities, unknown_entities);

    auto contains_unknown = !unknown_entities.empty();

    if (contains_unknown) {
        // Request unknown entities that haven't yet been requested.
        auto &ctx = registry.ctx<client_network_context>();
        auto req = packet::entity_request{};

        for (auto entity : unknown_entities) {
            if (!ctx.requested_entities.contains(entity)) {
                ctx.requested_entities.emplace(entity);
                req.entities.push_back(entity);
            }
        }

        if (!req.entities.empty()) {
            ctx.packet_signal.publish(packet::edyn_packet{std::move(req)});
        }
    }

    return contains_unknown;
}

static void insert_input_to_state_history(entt::registry &registry, const registry_snapshot &snap, double time) {
    auto &ctx = registry.ctx<client_network_context>();
    entt::sparse_set unwoned_entities;

    for (auto entity : snap.entities) {
        if (!ctx.owned_entities.contains(entity) && !unwoned_entities.contains(entity)) {
            unwoned_entities.emplace(entity);
        }
    }

    ctx.state_history->emplace(snap, unwoned_entities, time);
}

static void snap_to_transient_snapshot(entt::registry &registry, packet::transient_snapshot &snapshot) {
    // Collect all entities present in snapshot and find islands where they
    // reside and finally send the snapshot to the island workers.
    auto island_entities = collect_islands_from_residents(registry, snapshot.entities.begin(), snapshot.entities.end());
    auto &coordinator = registry.ctx<island_coordinator>();

    auto msg = msg::apply_network_pools{std::move(snapshot.entities), std::move(snapshot.pools)};

    for (auto island_entity : island_entities) {
        coordinator.send_island_message<msg::apply_network_pools>(island_entity, msg);
        coordinator.wake_up_island(island_entity);
    }
}

static void process_packet(entt::registry &registry, packet::transient_snapshot &snapshot) {
    auto contains_unknown_entities = request_unknown_entities(registry, snapshot.entities);

    if (contains_unknown_entities) {
        // Do not perform extrapolation if it contains unknown entities as the
        // result would not make much sense if all parts are not involved. Wait
        // until the entity request is completed and then extrapolations will
        // be performed normally again. This should not happen very often.
        return;
    }

    auto &ctx = registry.ctx<client_network_context>();
    auto &settings = registry.ctx<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    // Translate transient snapshot into client's space so entities in the
    // snapshot will make sense in this registry. This same snapshot will
    // be given to the extrapolation job, thus containing entities in the
    // main registry space.
    snapshot.convert_remloc(registry, ctx.entity_map);

    const auto time = performance_time();
    double snapshot_time;

    if (ctx.clock_sync.count > 0) {
        snapshot_time = snapshot.timestamp + ctx.clock_sync.time_delta - ctx.server_playout_delay;
    } else {
        const auto client_server_time_difference = ctx.server_playout_delay + client_settings.round_trip_time / 2;
        snapshot_time = time - client_server_time_difference;
    }

    // Input from other clients must be always added to the state history.
    // The server won't send input components of entities owned by this client.
    insert_input_to_state_history(registry, snapshot, snapshot_time);

    // Snap simulation to server state if the amount of time to be extrapolated
    // is smaller than the fixed delta time, which would cause the extrapolation
    // job to perform no physics steps anyways, within a certain threshold (if
    // the time difference nearly equals fixed dt, it is possible it would
    // perform a single step since time will have passed until the job starts
    // running).
    auto needs_extrapolation = time - snapshot_time > settings.fixed_dt;

    // If extrapolation is not enabled or not needed send the snapshot directly
    // to the island workers. They will snap to this state and add the
    // differences to the discontinuity components.
    if (!needs_extrapolation || !client_settings.extrapolation_enabled) {
        snap_to_transient_snapshot(registry, snapshot);
        return;
    }

    // Ignore it if the number of current extrapolation jobs is at maximum.
    if (ctx.extrapolation_jobs.size() >= client_settings.max_concurrent_extrapolations) {
        return;
    }

    // Collect all entities to be included in extrapolation, that is, basically
    // all entities in the transient snapshot packet and the edges connecting
    // them.
    auto entities = entt::sparse_set{};
    auto node_view = registry.view<graph_node>();
    auto &graph = registry.ctx<entity_graph>();

    for (auto entity : snapshot.entities) {
        entities.emplace(entity);

        if (node_view.contains(entity)) {
            auto node_index = node_view.get<graph_node>(entity).node_index;

            graph.visit_edges(node_index, [&] (auto edge_index) {
                auto edge_entities = graph.edge_node_entities(edge_index);
                auto other_entity = edge_entities.first == entity ? edge_entities.second : edge_entities.first;

                if (vector_contains(snapshot.entities, other_entity)) {
                    auto edge_entity = graph.edge_entity(edge_index);

                    if (!entities.contains(edge_entity)) {
                        entities.emplace(edge_entity);
                    }
                }
            });
        }
    }

    // TODO: only include the necessary static entities. Could extrapolate the
    // position by twice their velocity and calculate a sweep AABB (union of
    // initial and extrapolated AABB) and query the non-procedural broadphase
    // tree to obtain the relevant static and kinematic entities.
    for (auto entity : registry.view<static_tag>()) {
        if (!entities.contains(entity)) {
            entities.emplace(entity);
        }
    }

    // Create input to send to extrapolation job.
    extrapolation_input input;
    input.is_input_component_func = ctx.is_input_component_func;
    input.start_time = snapshot_time;

    for (auto entity : entities) {
        if (auto *owner = registry.try_get<entity_owner>(entity);
            owner && owner->client_entity == ctx.client_entity)
        {
            input.owned_entities.emplace(entity);
        }
    }

    auto builder = (*settings.make_reg_op_builder)();
    builder->create(entities.begin(), entities.end());
    builder->emplace_all(registry, entities);
    input.ops = builder->finish();

    input.entities = std::move(entities);
    input.transient_snapshot = std::move(snapshot);
    input.should_remap = true;

    // Create extrapolation job and put the registry snapshot and the transient
    // snapshot into its message queue.
    auto &material_table = registry.ctx<material_mix_table>();

    auto job = std::make_unique<extrapolation_job>(std::move(input), settings, material_table, ctx.state_history);
    job->reschedule();

    ctx.extrapolation_jobs.push_back(extrapolation_job_context{std::move(job)});
}

static void process_packet(entt::registry &registry, packet::general_snapshot &snapshot) {
    request_unknown_entities(registry, snapshot.entities);

    const auto time = performance_time();
    auto &settings = registry.ctx<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    auto &ctx = registry.ctx<client_network_context>();

    double snapshot_time;

    if (ctx.clock_sync.count > 0) {
        snapshot_time = snapshot.timestamp + ctx.clock_sync.time_delta - ctx.server_playout_delay;
    } else {
        const auto client_server_time_difference = ctx.server_playout_delay + client_settings.round_trip_time / 2;
        snapshot_time = time - client_server_time_difference;
    }

    snapshot.convert_remloc(registry, ctx.entity_map);
    insert_input_to_state_history(registry, snapshot, snapshot_time);
    const bool mark_dirty = true;

    ctx.snapshot_importer->import_local(registry, snapshot, mark_dirty);
}

static void process_packet(entt::registry &registry, packet::set_playout_delay &delay) {
    auto &ctx = registry.ctx<client_network_context>();
    ctx.server_playout_delay = delay.value;
}

static void process_packet(entt::registry &registry, const packet::time_request &req) {
    auto res = packet::time_response{req.id, performance_time()};
    auto &ctx = registry.ctx<client_network_context>();
    ctx.packet_signal.publish(packet::edyn_packet{res});
}

static void process_packet(entt::registry &registry, const packet::time_response &res) {
    auto &ctx = registry.ctx<client_network_context>();
    clock_sync_process_time_response(ctx.clock_sync, res);
}

static void process_packet(entt::registry &registry, const packet::server_settings &server) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.fixed_dt = server.fixed_dt;
    settings.gravity = server.gravity;
    settings.num_solver_velocity_iterations = server.num_solver_velocity_iterations;
    settings.num_solver_position_iterations = server.num_solver_position_iterations;
    settings.num_restitution_iterations = server.num_restitution_iterations;
    settings.num_individual_restitution_iterations = server.num_individual_restitution_iterations;
    registry.ctx<island_coordinator>().settings_changed();

    auto &ctx = registry.ctx<client_network_context>();
    ctx.allow_full_ownership = server.allow_full_ownership;
}

static void process_packet(entt::registry &, const packet::set_aabb_of_interest &) {}

void client_receive_packet(entt::registry &registry, packet::edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

bool client_owns_entity(const entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_network_context>();
    return ctx.client_entity == registry.get<entity_owner>(entity).client_entity;
}

}
