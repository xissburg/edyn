#include "edyn/networking/sys/server_side.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/asset_ref.hpp"
#include "edyn/networking/comp/asset_entry.hpp"
#include "edyn/networking/packet/asset_sync.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/packet/entity_entered.hpp"
#include "edyn/networking/packet/entity_exited.hpp"
#include "edyn/networking/packet/entity_response.hpp"
#include "edyn/networking/packet/query_entity.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/util/process_update_entity_map_packet.hpp"
#include "edyn/networking/util/snap_to_pool_snapshot.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/vector_util.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/time/simulation_time.hpp"
#include <entt/entity/registry.hpp>
#include <algorithm>
#include <set>

namespace edyn {

static void process_packet(entt::registry &registry, entt::entity client_entity, packet::registry_snapshot &snapshot) {
    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->send_message_to_worker<msg::apply_network_pools>(std::move(snapshot.entities), std::move(snapshot.pools), false);
    } else {
        snap_to_pool_snapshot(registry, snapshot.entities, snapshot.pools, false);
        wake_up_island_residents(registry, snapshot.entities);
    }
}

template<typename T>
void create_graph_edge(entt::registry &registry, entt::entity entity) {
    if (registry.any_of<graph_edge>(entity)) return;

    auto &comp = registry.get<T>(entity);
    auto node_index0 = registry.get<graph_node>(comp.body[0]).node_index;
    auto node_index1 = registry.get<graph_node>(comp.body[1]).node_index;
    auto edge_index = registry.ctx().get<entity_graph>().insert_edge(entity, node_index0, node_index1);
    registry.emplace<graph_edge>(entity, edge_index);
    registry.emplace<island_resident>(entity);
}

template<typename... Ts>
void maybe_create_graph_edge(entt::registry &registry, entt::entity entity) {
    ((registry.any_of<Ts>(entity) ? create_graph_edge<Ts>(registry, entity) : void(0)), ...);
}

template<typename... Ts>
void maybe_create_graph_edge(entt::registry &registry, entt::entity entity, [[maybe_unused]] std::tuple<Ts...>) {
    maybe_create_graph_edge<Ts...>(registry, entity);
}

static void process_packet(entt::registry &registry, entt::entity client_entity,
                           const packet::create_entity &packet) {
    auto &ctx = registry.ctx().get<server_network_context>();
    auto &client = registry.get<remote_client>(client_entity);

    // Collect entity mappings for new entities to send back to client.
    auto emap_packet = packet::update_entity_map{};

    // Create entities first, import pools later, since components might contain
    // entities which have to be mapped from remote to local.
    for (auto remote_entity : packet.entities) {
        if (client.entity_map.contains(remote_entity)) {
            continue;
        }

        auto local_entity = registry.create();
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push(local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        auto &settings = registry.ctx().get<edyn::settings>();
        emap_packet.timestamp = (*settings.time_func)();
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{emap_packet});
    }

    // Must not check ownership because entities are being created for the the
    // client thus all entities are already assumed to be owned by the client.
    // Also, checking ownership at this point would fail since nodes and edges
    // haven't yet been created and islands haven't been assigned.
    const bool check_ownership = false;
    ctx.snapshot_importer->import(registry, client_entity, packet, check_ownership);

    // Create nodes and edges in entity graph, assign networked tags and
    // dependent components which are not networked.
    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.at(remote_entity);

        // Assign computed properties such as AABB and inverse mass.
        if (registry.any_of<shape_index>(local_entity)) {
            auto &pos = registry.get<position>(local_entity);
            auto &orn = registry.get<orientation>(local_entity);

            visit_shape(registry, local_entity, [&](auto &&shape) {
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

        if (!registry.all_of<networked_tag>(local_entity)) {
            registry.emplace<networked_tag>(local_entity);
        }

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto is_procedural = registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx().get<entity_graph>().insert_node(local_entity, !is_procedural);
            registry.emplace<graph_node>(local_entity, node_index);

            if (is_procedural) {
                registry.emplace<island_resident>(local_entity);
            } else {
                registry.emplace<multi_island_resident>(local_entity);
            }
        }
    }

    // Create graph edges for constraints *after* graph nodes have been created
    // for rigid bodies above.
    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.at(remote_entity);
        maybe_create_graph_edge(registry, local_entity, constraints_tuple);
        maybe_create_graph_edge<null_constraint>(registry, local_entity);
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::destroy_entity &packet) {
    auto &client = registry.get<remote_client>(client_entity);
    auto &aabboi = registry.get<aabb_of_interest>(client_entity);

    for (auto remote_entity : packet.entities) {
        if (client.entity_map.contains(remote_entity)) {
            auto local_entity = client.entity_map.at(remote_entity);

            if (registry.valid(local_entity)) {
                auto *owner = registry.try_get<entity_owner>(local_entity);

                if (owner && owner->client_entity == client_entity) {
                    registry.destroy(local_entity);
                    client.entity_map.erase(remote_entity);
                    client.owned_entities.erase(local_entity);

                    // Remove from AABB of interest of owner to prevent notifying
                    // the requester itself of destruction of these entities.
                    aabboi.entities.remove(local_entity);
                }
            }
        }
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::update_entity_map &packet) {
    auto &client = registry.get<remote_client>(client_entity);
    process_update_entity_map_packet(registry, packet, client.entity_map);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::time_request &req) {
    auto &settings = registry.ctx().get<edyn::settings>();
    auto res = packet::time_response{req.id, (*settings.time_func)()};
    auto &ctx = registry.ctx().get<server_network_context>();
    ctx.packet_signal.publish(client_entity, packet::edyn_packet{res});
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::time_response &res) {
    auto &client = registry.get<remote_client>(client_entity);
    auto &settings = registry.ctx().get<edyn::settings>();
    const auto time = (*settings.time_func)();
    clock_sync_process_time_response(client.clock_sync, res, time);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::set_aabb_of_interest &aabb) {
    if (!(aabb.max > aabb.min)) {
        return;
    }

    auto &aabboi = registry.get<aabb_of_interest>(client_entity);
    aabboi.aabb.min = aabb.min;
    aabboi.aabb.max = aabb.max;
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::query_entity &query) {
    auto &ctx = registry.ctx().get<server_network_context>();
    auto res = packet::entity_response{};
    res.id = query.id;

    for (auto [entity, indices] : query.entities) {
        if (!indices.empty() && registry.valid(entity)) {
            ctx.snapshot_exporter->export_comp_index(res, entity, indices);
        }
    }

    // Sort components to ensure order of construction on the other end.
    std::sort(res.pools.begin(), res.pools.end(), [](auto &&lhs, auto &&rhs) {
        return lhs.component_index < rhs.component_index;
    });

    ctx.packet_signal.publish(client_entity, packet::edyn_packet{res});
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::asset_sync &query) {
    if (!registry.valid(query.entity) || !registry.all_of<asset_ref>(query.entity)) {
        return;
    }

    auto res = packet::asset_sync_response{};
    res.id = query.id;
    res.entity = query.entity;

    auto &ctx = registry.ctx().get<server_network_context>();
    auto entry_view = registry.view<asset_entry>();
    auto &asset = registry.get<asset_ref>(query.entity);

    for (auto [asset_id, entity] : asset.entity_map) {
        auto [entry] = entry_view.get(entity);
        if (!entry.sync_indices.empty()) {
            ctx.snapshot_exporter->export_comp_index(res, entity, entry.sync_indices);
        }
    }

    // Sort components to ensure order of construction on the other end.
    std::sort(res.pools.begin(), res.pools.end(), [](auto &&lhs, auto &&rhs) {
        return lhs.component_index < rhs.component_index;
    });

    ctx.packet_signal.publish(client_entity, packet::edyn_packet{res});
}

static void process_packet(entt::registry &, entt::entity, const packet::entity_response &) {}
static void process_packet(entt::registry &, entt::entity, const packet::client_created &) {}
static void process_packet(entt::registry &, entt::entity, const packet::set_playout_delay &) {}
static void process_packet(entt::registry &, entt::entity, const packet::server_settings &) {}
static void process_packet(entt::registry &, entt::entity, const packet::entity_entered &) {}
static void process_packet(entt::registry &, entt::entity, const packet::entity_exited &) {}
static void process_packet(entt::registry &, entt::entity, const packet::asset_sync_response &) {}

void init_network_server(entt::registry &registry) {
    registry.ctx().emplace<server_network_context>(registry);

    auto &settings = registry.ctx().get<edyn::settings>();
    settings.network_settings = server_network_settings{};
}

void deinit_network_server(entt::registry &registry) {
    registry.ctx().erase<server_network_context>();

    auto &settings = registry.ctx().get<edyn::settings>();
    settings.network_settings = {};
}

static void server_process_timed_packets(entt::registry &registry, double time) {
    registry.view<remote_client>().each([&](entt::entity client_entity, remote_client &client) {
        auto it = client.packet_queue.begin();

        for (; it != client.packet_queue.end(); ++it) {
            if (it->timestamp > time - client.playout_delay) {
                break;
            }

            std::visit([&](auto &&packet) {
                process_packet(registry, client_entity, packet);
            }, it->packet.var);
        }

        client.packet_queue.erase(client.packet_queue.begin(), it);
    });
}

static void publish_pending_created_clients(entt::registry &registry) {
    auto &settings = registry.ctx().get<edyn::settings>();
    auto client_view = registry.view<remote_client>();
    auto &ctx = registry.ctx().get<server_network_context>();

    for (auto client_entity : ctx.pending_created_clients) {
        if (!registry.valid(client_entity)) {
            continue;
        }

        auto packet = packet::client_created{client_entity};
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});

        auto [client] = client_view.get(client_entity);
        auto settings_packet = packet::server_settings(settings, client.allow_full_ownership);
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{settings_packet});
    }

    ctx.pending_created_clients.clear();
}

static void process_aabb_of_interest_entities_exited(entt::registry &registry,
                                                     entt::entity client_entity,
                                                     aabb_of_interest &aabboi) {
    if (aabboi.entities_exited.empty()) {
        return;
    }

    // Notify client of entities that have been removed from its AABB-of-interest.
    // Add assets separately. Do not include entities that belong to an asset.
    // With the asset entity the client will be able to delete all entities that
    // belong to it.
    entt::sparse_set assets;
    std::vector<entt::entity> entities;
    auto &client = registry.get<remote_client>(client_entity);

    const auto entry_view = registry.view<asset_entry>();
    for (auto entity : aabboi.entities_exited) {
        if (entry_view.contains(entity)) {
            auto [entry] = entry_view.get(entity);

            if (!assets.contains(entry.asset_entity)) {
                assets.push(entry.asset_entity);

                if (client.entity_map.contains_local(entry.asset_entity)) {
                    client.entity_map.erase_local(entry.asset_entity);
                }
            }
        } else {
            entities.push_back(entity);
        }

        if (client.entity_map.contains_local(entity)) {
            client.entity_map.erase_local(entity);
        }
    }

    auto packet = packet::entity_exited{};
    packet.entities = std::move(entities);
    packet.entities.insert(packet.entities.end(), assets.begin(), assets.end());

    auto &ctx = registry.ctx().get<server_network_context>();
    ctx.packet_signal.publish(client_entity, packet::edyn_packet{std::move(packet)});

    // Do not forget to clear it after processing.
    aabboi.entities_exited.clear();
}

static void process_aabb_of_interest_entities_entered(entt::registry &registry,
                                                      entt::entity client_entity,
                                                      aabb_of_interest &aabboi) {
    if (aabboi.entities_entered.empty()) {
        return;
    }

    const auto entry_view = registry.view<asset_entry>();
    entt::sparse_set assets;
    entt::sparse_set entities;

    for (auto entity : aabboi.entities_entered) {
        if (entry_view.contains(entity)) {
            auto [entry] = entry_view.get(entity);

            if (!assets.contains(entry.asset_entity)) {
                assets.push(entry.asset_entity);
            }
        } else {
            entities.push(entity);
        }
    }

    if (!assets.empty()) {
        auto &ctx = registry.ctx().get<server_network_context>();
        auto asset_view = registry.view<asset_ref>();
        auto owner_view = registry.view<entity_owner>();
        auto entry_view = registry.view<asset_entry>();
        auto packet = packet::entity_entered{};

        for (auto asset_entity : assets) {
            auto &info = packet.entry.emplace_back();
            info.entity = asset_entity;
            info.asset = std::get<0>(asset_view.get(asset_entity));
            info.owner = owner_view.contains(asset_entity) ?
                         std::get<0>(owner_view.get(asset_entity)).client_entity : entt::null;

            for (auto [asset_id, entity] : info.asset.entity_map) {
                auto [entry] = entry_view.get(entity);
                if (!entry.sync_indices.empty()) {
                    ctx.snapshot_exporter->export_comp_index(info, entity, entry.sync_indices);
                }
            }

            // Sort components to ensure order of construction on the other end.
            std::sort(info.pools.begin(), info.pools.end(), [](auto &&lhs, auto &&rhs) {
                return lhs.component_index < rhs.component_index;
            });
        }

        ctx.packet_signal.publish(client_entity, packet::edyn_packet{std::move(packet)});
    }

    if (!entities.empty()) {
        auto packet = packet::create_entity{};

        auto &ctx = registry.ctx().get<server_network_context>();
        ctx.snapshot_exporter->export_all(packet, entities);

        // Sort components to ensure order of construction on the other end.
        std::sort(packet.pools.begin(), packet.pools.end(), [](auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        ctx.packet_signal.publish(client_entity, packet::edyn_packet{std::move(packet)});
    }

    // Do not forget to clear it after processing.
    aabboi.entities_entered.clear();
}

static void maybe_publish_client_registry_snapshot(entt::registry &registry,
                                                   entt::entity client_entity,
                                                   remote_client &client,
                                                   aabb_of_interest &aabboi,
                                                   double time) {
    if (time - client.last_snapshot_time < 1 / client.snapshot_rate) {
        return;
    }

    client.last_snapshot_time = time;

    auto &ctx = registry.ctx().get<server_network_context>();
    auto packet = packet::registry_snapshot{};
    ctx.snapshot_exporter->export_modified(packet, aabboi.entities, client_entity);

    if (!packet.entities.empty() && !packet.pools.empty()) {
        packet.timestamp = get_simulation_timestamp(registry);
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});
    }
}

static void calculate_client_playout_delay(entt::registry &registry,
                                           entt::entity client_entity,
                                           remote_client &client,
                                           aabb_of_interest &aabboi) {
    auto owner_view = registry.view<entity_owner>();
    auto client_view = registry.view<remote_client>();
    auto biggest_rtt = client.round_trip_time;

    for (auto entity : aabboi.entities) {
        if (!owner_view.contains(entity)) {
            continue;
        }

        auto [owner] = owner_view.get(entity);
        auto [other_client] = client_view.get(owner.client_entity);
        biggest_rtt = std::max(other_client.round_trip_time, biggest_rtt);
    }

    auto &settings = registry.ctx().get<edyn::settings>();
    auto &server_settings = std::get<server_network_settings>(settings.network_settings);
    auto biggest_latency = biggest_rtt / 2;
    auto playout_delay = std::min(biggest_latency * server_settings.playout_delay_multiplier,
                                  server_settings.max_playout_delay);

    // Update playout delay if the difference is of significance.
    if (std::abs(playout_delay - client.playout_delay) > client.playout_delay * 0.06) {
        client.playout_delay = playout_delay;

        auto packet = edyn::packet::set_playout_delay{playout_delay};
        auto &ctx = registry.ctx().get<server_network_context>();
        ctx.packet_signal.publish(client_entity, edyn::packet::edyn_packet{packet});
    }
}

static void process_aabbs_of_interest(entt::registry &registry, double time) {
    for (auto [client_entity, client, aabboi] : registry.view<remote_client, aabb_of_interest>().each()) {
        process_aabb_of_interest_entities_exited(registry, client_entity, aabboi);
        process_aabb_of_interest_entities_entered(registry, client_entity, aabboi);
        maybe_publish_client_registry_snapshot(registry, client_entity, client, aabboi, time);
        calculate_client_playout_delay(registry, client_entity, client, aabboi);
    }
}

static void server_update_clock_sync(entt::registry &registry, double time) {
    for (auto [client_entity, client] : registry.view<remote_client>().each()) {
        update_clock_sync(client.clock_sync, time, client.round_trip_time);
    };
}

static void dispatch_actions(entt::registry &registry, double time) {
    auto &ctx = registry.ctx().get<server_network_context>();
    auto client_view = registry.view<remote_client>();

    // Consume actions with a timestamp that's before the current execution time.
    for (auto [entity, history, owner] : registry.view<action_history, entity_owner>().each()) {
        if (history.entries.empty()) {
            continue;
        }

        auto [client] = client_view.get(owner.client_entity);
        auto it = history.entries.begin();
        auto last_timestamp = client.last_executed_history_entry_timestamp;

        for (; it != history.entries.end(); ++it) {
            if (it->timestamp <= client.last_executed_history_entry_timestamp) {
                // Action has already been processed.
                continue;
            }

            if (it->timestamp > time - client.playout_delay) {
                // This action and all that follow should be processed later.
                break;
            }

            ctx.snapshot_importer->import_action(registry, entity, it->action_index, it->data);
            last_timestamp = it->timestamp;
        }

        client.last_executed_history_entry_timestamp = last_timestamp;

        // Delete all actions up to the last one that was executed.
        history.entries.erase(history.entries.begin(), it);
    }
}

void update_server_snapshot_exporter(entt::registry &registry, double time) {
    auto &ctx = registry.ctx().get<server_network_context>();
    ctx.snapshot_exporter->update(time);
}

void update_network_server(entt::registry &registry) {
    auto &settings = registry.ctx().get<edyn::settings>();
    const auto time = (*settings.time_func)();
    server_update_clock_sync(registry, time);
    server_process_timed_packets(registry, time);
    update_server_snapshot_exporter(registry, time);
    update_aabbs_of_interest(registry);
    process_aabbs_of_interest(registry, time);
    publish_pending_created_clients(registry);
    dispatch_actions(registry, time);
}

template<typename T>
void insert_packet_to_queue(remote_client &client, double timestamp, T &&packet) {
    // Sorted insertion.
    auto insert_it = std::find_if(client.packet_queue.begin(), client.packet_queue.end(),
                                  [timestamp](auto &&p) { return p.timestamp > timestamp; });
    client.packet_queue.insert(insert_it, timed_packet{timestamp, packet::edyn_packet{std::move(packet)}});
}

template<typename T>
void enqueue_packet(entt::registry &registry, entt::entity client_entity, T &packet, double time) {
    auto &client = registry.get<remote_client>(client_entity);
    double packet_timestamp;

    if (client.clock_sync.count > 0) {
        packet_timestamp = std::min(packet.timestamp + client.clock_sync.time_delta, time);
    } else {
        packet_timestamp = time - client.round_trip_time / 2;
    }

    insert_packet_to_queue(client, packet_timestamp, packet);
}

template<>
void enqueue_packet<packet::registry_snapshot>(entt::registry &registry, entt::entity client_entity,
                                               packet::registry_snapshot &packet, double time) {
    // Specialize it for registry snapshots. Transform entities to local and
    // import action history in advance.
    auto &client = registry.get<remote_client>(client_entity);
    double time_delta;

    if (client.clock_sync.count > 0) {
        time_delta = client.clock_sync.time_delta;
    } else {
        time_delta = time - (packet.timestamp + client.round_trip_time / 2);
    }

    for (auto &entity : packet.entities) {
        // Discard packet if it contains unknown entities.
        if (!client.entity_map.contains(entity)) {
            return;
        }
        entity = client.entity_map.at(entity);
    }

    // Transform snapshot entities into local registry space and then import
    // action history.
    auto &ctx = registry.ctx().get<server_network_context>();
    const bool check_ownership = true;
    ctx.snapshot_importer->transform_to_local(registry, client_entity, packet, check_ownership);
    ctx.snapshot_importer->merge_action_history(registry, packet, time_delta);

    // The action history pool is removed from the packet after being merged.
    // Do not enqueue if there are no other pools in the packet.
    if (!packet.pools.empty()) {
        double packet_timestamp;

        if (client.clock_sync.count > 0) {
            packet_timestamp = std::min(packet.timestamp + client.clock_sync.time_delta, time);
        } else {
            packet_timestamp = time - client.round_trip_time / 2;
        }

        insert_packet_to_queue(client, packet_timestamp, packet);
    }
}

void server_receive_packet(entt::registry &registry, entt::entity client_entity, packet::edyn_packet &packet) {
    std::visit([&](auto &&decoded_packet) {
        using PacketType = std::decay_t<decltype(decoded_packet)>;
        // If it's a timed packet, enqueue for later execution. Process
        // immediately otherwise.
        if constexpr(tuple_has_type<PacketType, packet::timed_packets_tuple_t>::value) {
            auto &settings = registry.ctx().get<edyn::settings>();
            const auto time = (*settings.time_func)();
            enqueue_packet(registry, client_entity, decoded_packet, time);
        } else {
            process_packet(registry, client_entity, decoded_packet);
        }
    }, packet.var);
}

// Local struct to be connected to the clock sync packet signal. This is
// necessary so the client entity can be passed to the context packet signal.
struct client_packet_signal_wrapper {
    server_network_context *ctx;
    entt::entity client_entity;

    // Make this a stable component in the EnTT storage.
    static constexpr auto in_place_delete = true;

    void publish(const packet::edyn_packet &packet) {
        ctx->packet_signal.publish(client_entity, packet);
    }
};

void server_make_client(entt::registry &registry, entt::entity entity, bool allow_full_ownership) {
    auto &ctx = registry.ctx().get<server_network_context>();

    auto &client = registry.emplace<remote_client>(entity);
    client.allow_full_ownership = allow_full_ownership;
    registry.emplace<aabb_of_interest>(entity);

    // Assign packet signal wrapper as a component since the `entt::delegate`
    // stores a reference to the `value_or_instance` parameter.
    auto &wrapper = registry.emplace<client_packet_signal_wrapper>(entity, &ctx, entity);
    client.clock_sync.send_packet.connect<&client_packet_signal_wrapper::publish>(wrapper);

    // `client_created` packets aren't published here at client construction
    // because at this point the caller wouldn't have a chance to receive the
    // packet as a signal in client's packet sink. Thus, this packet is
    // published later on a call to `update_network_server`.
    ctx.pending_created_clients.push_back(entity);
}

entt::entity server_make_client(entt::registry &registry, bool allow_full_ownership) {
    auto entity = registry.create();
    server_make_client(registry, entity, allow_full_ownership);
    return entity;
}

void server_destroy_client(entt::registry &registry, entt::entity client_entity) {
    auto &client = registry.get<remote_client>(client_entity);

    for (auto entity : client.owned_entities) {
        if (registry.valid(entity)) {
            registry.destroy(entity);
        }
    }

    registry.destroy(client_entity);
}

void server_set_allow_full_ownership(entt::registry &registry, entt::entity client_entity, bool allow_full_ownership) {
    auto &client = registry.get<remote_client>(client_entity);
    client.allow_full_ownership = allow_full_ownership;
    // TODO notify client
}

void server_set_client_round_trip_time(entt::registry &registry, entt::entity client_entity, double rtt) {
    auto &client = registry.get<remote_client>(client_entity);
    client.round_trip_time = rtt;
}

}
