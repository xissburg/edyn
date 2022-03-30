#include "edyn/networking/sys/server_side.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/packet/general_snapshot.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/util/process_update_entity_map_packet.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/edyn.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/vector.hpp"
#include "edyn/util/aabb_util.hpp"
#include <entt/entity/registry.hpp>
#include <algorithm>
#include <set>

namespace edyn {

static void update_island_entity_owners(entt::registry &registry) {
    // The client has ownership of their entities if they're the only client in
    // the island where the entity resides. They're also granted temporary
    // ownership of all other entities in that island.
    auto owner_view = registry.view<entity_owner>();

    for (auto [island_entity, island, island_owner] : registry.view<island, entity_owner>().each()) {
        // Set island owner to null and find out whether it can have a single owner.
        island_owner.client_entity = entt::null;

        for (auto it = island.nodes.begin(); it != island.edges.end(); ++it) {
            if (it == island.nodes.end()) {
                it = island.edges.begin();
            }

            auto entity = *it;

            if (!owner_view.contains(entity)) {
                continue;
            }

            auto [owner] = owner_view.get(entity);

            if (owner.client_entity == entt::null) {
                continue;
            }

            if (island_owner.client_entity == entt::null) {
                // Island is not owned by any client yet, thus assign this
                // client as the owner.
                island_owner.client_entity = owner.client_entity;
            } else if (island_owner.client_entity != owner.client_entity) {
                // Island contains more than one client in it, thus it cannot
                // be owned by either.
                island_owner.client_entity = entt::null;
                break;
            }
        }
    }
}

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity) {
    auto &client = registry.get<remote_client>(client_entity);

    if (!client.allow_full_ownership) {
        return false;
    }

    auto owner_view = registry.view<entity_owner>();

    if (auto *resident = registry.try_get<island_resident>(entity)) {
        auto [island_owner] = owner_view.get(resident->island_entity);
        return island_owner.client_entity == client_entity;
    } else if (auto *resident = registry.try_get<multi_island_resident>(entity)) {
        for (auto island_entity : resident->island_entities) {
            auto [island_owner] = owner_view.get(island_entity);

            if (island_owner.client_entity != client_entity) {
                return false;
            }
        }

        return true;
    }

    return true;
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_request &req) {
    entt::sparse_set valid_entities;

    for (auto entity : req.entities) {
        if (registry.valid(entity) && !valid_entities.contains(entity)) {
            valid_entities.emplace(entity);
        }
    }

    // Send back all entities in all islands associated with the request
    // entities. This is necessary because otherwise the response could be
    // incomplete, e.g. dependent entities won't be sent which will result
    // in an import failure on the other end.
    auto island_entities = collect_islands_from_residents(registry, valid_entities.begin(), valid_entities.end());
    auto island_view = registry.view<island>();
    auto networked_view = registry.view<networked_tag>();
    entt::sparse_set all_entities;

    for (auto island_entity : island_entities) {
        auto [island] = island_view.get(island_entity);

        for (auto entity : island.nodes) {
            if (!all_entities.contains(entity) && networked_view.contains(entity)) {
                all_entities.emplace(entity);
            }
        }

        for (auto entity : island.edges) {
            if (!all_entities.contains(entity) && networked_view.contains(entity)) {
                all_entities.emplace(entity);
            }
        }
    }

    // Include client entities of all entity owners.
    entt::sparse_set client_entities;
    auto owner_view = registry.view<entity_owner>();

    for (auto entity : all_entities) {
        if (owner_view.contains(entity)) {
            auto [owner] = owner_view.get(entity);

            if (!client_entities.contains(owner.client_entity)) {
                client_entities.emplace(owner.client_entity);
            }
        }
    }

    auto &ctx = registry.ctx<server_network_context>();
    auto res = packet::entity_response{};
    res.entities.insert(res.entities.end(), all_entities.begin(), all_entities.end());
    res.entities.insert(res.entities.end(), client_entities.begin(), client_entities.end());

    ctx.snapshot_exporter->export_all(registry, res);

    if (!res.entities.empty() && !res.pools.empty()) {
        std::sort(res.pools.begin(), res.pools.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        ctx.packet_signal.publish(client_entity, packet::edyn_packet{res});
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_response &res) {
    // TODO: import requested entities.
}

static void process_packet(entt::registry &registry, entt::entity client_entity, packet::transient_snapshot &snapshot) {
    auto &ctx = registry.ctx<server_network_context>();
    const bool check_ownership = true;
    const bool mark_dirty = false;
    auto &client = registry.get<remote_client>(client_entity);

    for (auto &entity : snapshot.entities) {
        entity = client.entity_map.contains(entity) ? client.entity_map.at(entity) : entt::entity{entt::null};
    }

    // Transform snapshot entities into local registry space.
    ctx.snapshot_importer->transform_to_local(registry, client_entity, snapshot, check_ownership);

    // If this pool holds input components, import them directly into
    // the main registry.
    ctx.snapshot_importer->import_input_local(registry, client_entity, snapshot, mark_dirty);

    // Get islands of all entities contained in transient snapshot and send the
    // snapshot to them. They will import the pre-processed state into their
    // registries. Later, these components will be updated in the main registry
    // via a registry snapshot.
    auto island_entities = collect_islands_from_residents(registry, snapshot.entities.begin(), snapshot.entities.end());
    auto &coordinator = registry.ctx<island_coordinator>();
    auto msg = msg::apply_network_pools{std::move(snapshot.entities), std::move(snapshot.pools)};

    for (auto island_entity : island_entities) {
        coordinator.send_island_message<msg::apply_network_pools>(island_entity, msg);
        coordinator.wake_up_island(island_entity);
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::general_snapshot &snapshot) {
    auto &ctx = registry.ctx<server_network_context>();
    const bool check_ownership = true;
    const bool mark_dirty = true;
    ctx.snapshot_importer->import(registry, client_entity, snapshot, check_ownership, mark_dirty);
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

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<server_network_context>();
    auto &client = registry.get<remote_client>(client_entity);

    // Collect entity mappings for new entities to send back to client.
    auto emap_packet = packet::update_entity_map{};
    emap_packet.timestamp = performance_time();

    // Create entities first, import pools later, since components might contain
    // entities which have to be mapped from remote to local.
    for (auto remote_entity : packet.entities) {
        if (client.entity_map.contains(remote_entity)) continue;

        auto local_entity = registry.create();
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push_back(local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{emap_packet});
    }

    // Must not check ownership because entities are being created for the the
    // client thus all entities are already assumed to be owned by the client.
    // Also, checking ownership at this point would fail since nodes and edges
    // haven't yet been created and islands haven't been assigned.
    const bool check_ownership = false;

    // Do not mark components as dirty because they will already be sent with
    // the new entities to island workers thus marking them as dirty would
    // cause them to be created twice in the worker.
    const bool mark_dirty = false;
    ctx.snapshot_importer->import(registry, client_entity, packet, check_ownership, mark_dirty);

    // Create nodes and edges in entity graph, assign networked tags and
    // dependent components which are not networked.
    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.at(remote_entity);

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

        if (!registry.all_of<networked_tag>(local_entity)) {
            registry.emplace<networked_tag>(local_entity);
        }

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }
    }

    // Create graph edges for constraints *after* graph nodes have been created
    // for rigid bodies above.
    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.at(remote_entity);
        maybe_create_graph_edge(registry, local_entity, constraints_tuple);
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
                    vector_erase(client.owned_entities, local_entity);

                    // Remove from AABB of interest of owner to prevent notifying
                    // the requester itself of destruction of these entities.
                    if (aabboi.entities.contains(local_entity)) {
                        aabboi.entities.erase(local_entity);
                    }
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
    auto res = packet::time_response{req.id, performance_time()};
    auto &ctx = registry.ctx<server_network_context>();
    ctx.packet_signal.publish(client_entity, packet::edyn_packet{res});
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::time_response &res) {
    auto &client = registry.get<remote_client>(client_entity);
    clock_sync_process_time_response(client.clock_sync, res);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::set_aabb_of_interest &aabb) {
    if (!(aabb.max > aabb.min)) {
        return;
    }

    auto &aabboi = registry.get<aabb_of_interest>(client_entity);
    aabboi.aabb.min = aabb.min;
    aabboi.aabb.max = aabb.max;
}

static void process_packet(entt::registry &, entt::entity, const packet::client_created &) {}
static void process_packet(entt::registry &, entt::entity, const packet::set_playout_delay &) {}
static void process_packet(entt::registry &, entt::entity, const packet::server_settings &) {}

void init_network_server(entt::registry &registry) {
    registry.set<server_network_context>();
    // Assign an entity owner to every island created.
    registry.on_construct<island>().connect<&entt::registry::emplace<entity_owner>>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.network_settings = server_network_settings{};
}

void deinit_network_server(entt::registry &registry) {
    registry.unset<server_network_context>();
    registry.on_construct<island>().disconnect<&entt::registry::emplace<entity_owner>>();

    auto &settings = registry.ctx<edyn::settings>();
    settings.network_settings = {};
}

static void server_process_timed_packets(entt::registry &registry, double time) {
    registry.view<remote_client>().each([&] (entt::entity client_entity, remote_client &client) {
        auto it = client.packet_queue.begin();

        for (; it != client.packet_queue.end(); ++it) {
            if (it->timestamp > time - client.playout_delay) {
                break;
            }

            std::visit([&] (auto &&packet) {
                using PacketType = std::decay_t<decltype(packet)>;

                if constexpr(has_type<PacketType, packet::timed_packets_tuple_t>::value) {
                    process_packet(registry, client_entity, packet);
                }
            }, it->packet.var);
        }

        client.packet_queue.erase(client.packet_queue.begin(), it);
    });
}

static void publish_pending_created_clients(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    auto client_view = registry.view<remote_client>();
    auto &ctx = registry.ctx<server_network_context>();

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

static void publish_client_current_snapshots(entt::registry &registry) {
    auto &ctx = registry.ctx<server_network_context>();

    // Send out accumulated changes to clients.
    registry.view<remote_client>().each([&] (entt::entity client_entity, remote_client &client) {
        if (client.current_snapshot.pools.empty()) return;
        auto packet = packet::edyn_packet{std::move(client.current_snapshot)};
        ctx.packet_signal.publish(client_entity, packet);
        EDYN_ASSERT(client.current_snapshot.pools.empty());
    });
}

static void process_aabb_of_interest_destroyed_entities(entt::registry &registry,
                                                        entt::entity client_entity,
                                                        remote_client &client,
                                                        aabb_of_interest &aabboi,
                                                        double time) {
    if (aabboi.destroy_entities.empty()) {
        return;
    }

    // Notify client of entities that have been removed from its AABB-of-interest.
    auto owner_view = registry.view<entity_owner>();
    auto packet = packet::destroy_entity{};
    packet.timestamp = time;

    for (auto entity : aabboi.destroy_entities) {
        // Ignore entities owned by client.
        if (!registry.valid(entity) ||
            !owner_view.contains(entity) ||
            std::get<0>(owner_view.get(entity)).client_entity != client_entity)
        {
            packet.entities.push_back(entity);
        }
    }

    aabboi.destroy_entities.clear();

    if (!packet.entities.empty()) {
        auto &ctx = registry.ctx<server_network_context>();
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});
    }
}

static void process_aabb_of_interest_created_entities(entt::registry &registry,
                                                      entt::entity client_entity,
                                                      remote_client &client,
                                                      aabb_of_interest &aabboi,
                                                      double time) {
    if (aabboi.create_entities.empty()) {
        return;
    }

    auto owner_view = registry.view<entity_owner>();
    auto packet = packet::create_entity{};
    packet.timestamp = time;

    for (auto entity : aabboi.create_entities) {
        // Ignore entities owned by client, since these entities must be
        // persistent in the client-side.
        if (!owner_view.contains(entity) ||
            std::get<0>(owner_view.get(entity)).client_entity != client_entity)
        {
            packet.entities.push_back(entity);
        }
    }

    if (!packet.entities.empty()) {
        auto &ctx = registry.ctx<server_network_context>();
        ctx.snapshot_exporter->export_all(registry, packet);

        // Sort components to ensure order of construction on the other end.
        std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});
    }

    aabboi.create_entities.clear();
}

static void maybe_publish_client_transient_snapshot(entt::registry &registry,
                                                    entt::entity client_entity,
                                                    remote_client &client,
                                                    aabb_of_interest &aabboi,
                                                    double time) {
    if (time - client.last_snapshot_time < 1 / client.snapshot_rate) {
        return;
    }

    client.last_snapshot_time = time;

    auto &ctx = registry.ctx<server_network_context>();
    auto packet = packet::transient_snapshot{};
    packet.timestamp = time;

    // Only include entities which are in islands not fully owned by the client
    // since the server allows the client to have full control over entities in
    // the islands where there are no other clients present.
    auto should_include = [&] (entt::entity entity) {
        return
            !registry.any_of<sleeping_tag, static_tag>(entity) &&
            registry.all_of<networked_tag>(entity) &&
            !is_fully_owned_by_client(registry, client_entity, entity) &&
            ctx.snapshot_exporter->contains_transient(registry, entity);
    };

    for (auto entity : aabboi.entities) {
        if (should_include(entity)) {
            packet.entities.push_back(entity);
        }
    }

    ctx.snapshot_exporter->export_transient(registry, packet, client_entity);

    if (!packet.pools.empty()) {
        ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});
    }
}

static void publish_client_dirty_components(entt::registry &registry,
                                            entt::entity client_entity,
                                            remote_client &client,
                                            aabb_of_interest &aabboi,
                                            double time) {
    // Share dirty entity updates.
    auto packet = packet::general_snapshot{};
    packet.timestamp = time;

    auto &ctx = registry.ctx<server_network_context>();
    auto networked_view = registry.view<networked_tag>();
    auto dirty_view = registry.view<dirty>();
    auto network_dirty_view = registry.view<network_dirty>();

    auto should_include_entity = [&] (entt::entity entity) {
        return
            networked_view.contains(entity) &&
            (dirty_view.contains(entity) ||
             (network_dirty_view.contains(entity) && !is_fully_owned_by_client(registry, client_entity, entity)));
    };

    for (auto entity : aabboi.entities) {
        if (should_include_entity(entity)) {
            packet.entities.push_back(entity);
        }
    }

    for (auto entity : packet.entities) {
        // Add dirty components to snapshot, including for entities
        // owned by the destination client. This does not include components
        // marked as dirty during import of other snapshots since
        // `network_dirty` is used in `server_snapshot_importer` instead.
        if (dirty_view.contains(entity)) {
            auto [dirty] = dirty_view.get(entity);
            ctx.snapshot_exporter->export_dirty_steady(registry, entity, dirty, packet, client_entity);
        }

        // For the components that were marked dirty during a snapshot import,
        // only include updates for those not owned by this client, since that
        // would cause the state that was set by the client to be sent back to
        // the client itself. Transient components are ignored since they're
        // frequently updated via transient snapshots.
        if (network_dirty_view.contains(entity)) {
            auto [dirty] = network_dirty_view.get(entity);
            ctx.snapshot_exporter->export_dirty_steady(registry, entity, dirty, packet, client_entity);
        }
    }

    if (!packet.entities.empty() && !packet.pools.empty()) {
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

    auto &settings = registry.ctx<edyn::settings>();
    auto &server_settings = std::get<server_network_settings>(settings.network_settings);
    auto biggest_latency = biggest_rtt / 2;
    auto playout_delay = std::min(biggest_latency * server_settings.playout_delay_multiplier,
                                  server_settings.max_playout_delay);

    // Update playout delay if the difference is of significance.
    if (std::abs(playout_delay - client.playout_delay) > 0.002) {
        client.playout_delay = playout_delay;

        auto packet = edyn::packet::set_playout_delay{playout_delay};
        auto &ctx = registry.ctx<server_network_context>();
        ctx.packet_signal.publish(client_entity, edyn::packet::edyn_packet{packet});
    }
}

static void merge_network_dirty_into_dirty(entt::registry &registry) {
    // Merge components marked as dirty during network import (i.e.
    // `ctx.snapshot_importer->import(...)`) into the regular dirty
    // components so these changes will be pushed into the respective
    // island workers.
    for (auto [entity, network_dirty] : registry.view<edyn::network_dirty>().each()) {
        registry.get_or_emplace<edyn::dirty>(entity).merge(network_dirty);
    }

    // Clear dirty after processing.
    registry.clear<network_dirty>();
}

static void process_aabbs_of_interest(entt::registry &registry, double time) {
    for (auto [client_entity, client, aabboi] : registry.view<remote_client, aabb_of_interest>().each()) {
        process_aabb_of_interest_destroyed_entities(registry, client_entity, client, aabboi, time);
        process_aabb_of_interest_created_entities(registry, client_entity, client, aabboi, time);
        maybe_publish_client_transient_snapshot(registry, client_entity, client, aabboi, time);
        publish_client_dirty_components(registry, client_entity, client, aabboi, time);
        calculate_client_playout_delay(registry, client_entity, client, aabboi);
    }
}

static void server_update_clock_sync(entt::registry &registry, double time) {
    for (auto [client_entity, client] : registry.view<remote_client>().each()) {
        update_clock_sync(client.clock_sync, time, client.round_trip_time);
    };
}

void update_network_server(entt::registry &registry) {
    auto time = performance_time();
    server_update_clock_sync(registry, time);
    server_process_timed_packets(registry, time);
    update_island_entity_owners(registry);
    update_aabbs_of_interest(registry);
    process_aabbs_of_interest(registry, time);
    publish_pending_created_clients(registry);
    publish_client_current_snapshots(registry);
    merge_network_dirty_into_dirty(registry);
}

template<typename T>
void enqueue_packet(entt::registry &registry, entt::entity client_entity, T &&packet) {
    auto &client = registry.get<remote_client>(client_entity);
    double packet_timestamp;
    auto time = performance_time();

    if (client.clock_sync.count > 0) {
        packet_timestamp = std::min(packet.timestamp + client.clock_sync.time_delta, time);
    } else {
        packet_timestamp = time - client.round_trip_time / 2;
    }

    // Sorted insertion.
    auto insert_it = std::find_if(client.packet_queue.begin(), client.packet_queue.end(),
                                  [packet_timestamp] (auto &&p) { return p.timestamp > packet_timestamp; });
    client.packet_queue.insert(insert_it, timed_packet{packet_timestamp, packet::edyn_packet{std::move(packet)}});
}

void server_receive_packet(entt::registry &registry, entt::entity client_entity, packet::edyn_packet &packet) {
    std::visit([&] (auto &&decoded_packet) {
        using PacketType = std::decay_t<decltype(decoded_packet)>;
        // If it's a timed packet, enqueue for later execution. Process
        // immediately otherwise.
        if constexpr(has_type<PacketType, packet::timed_packets_tuple_t>::value) {
            enqueue_packet(registry, client_entity, decoded_packet);
        } else {
            process_packet(registry, client_entity, decoded_packet);
        }
    }, packet.var);
}

// Local struct to be connected to the clock sync send packet signal. This is
// necessary so the client entity can be passed to the context packet signal.
struct client_packet_signal_wrapper {
    server_network_context *ctx;
    entt::entity client_entity;

    void publish(const packet::edyn_packet &packet) {
        ctx->packet_signal.publish(client_entity, packet);
    }
};

void server_make_client(entt::registry &registry, entt::entity entity, bool allow_full_ownership) {
    auto &ctx = registry.ctx<server_network_context>();

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

void server_set_client_round_trip_time(entt::registry &registry, entt::entity client_entity, double rtt) {
    auto &client = registry.get<remote_client>(client_entity);
    client.round_trip_time = rtt;
}

void server_notify_created_entities(entt::registry &registry,
                                    entt::entity client_entity,
                                    const std::vector<entt::entity> &entities) {
    auto &ctx = registry.ctx<server_network_context>();

#ifdef EDYN_DEBUG
    // Ensure all entities are networked.
    for (auto entity : entities) {
        EDYN_ASSERT(registry.all_of<networked_tag>(entity));
    }
#endif

    auto packet = edyn::packet::create_entity{};
    packet.timestamp = performance_time();
    packet.entities = entities;
    ctx.snapshot_exporter->export_all(registry, packet);

    // Sort components to ensure order of construction.
    std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
        return lhs.component_index < rhs.component_index;
    });

    ctx.packet_signal.publish(client_entity, packet::edyn_packet{packet});
}

}
