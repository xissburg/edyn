#ifndef EDYN_NETWORKING_SERVER_SIDE_HPP
#define EDYN_NETWORKING_SERVER_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

void init_network_server(entt::registry &);
void deinit_network_server(entt::registry &);
void server_process_packets(entt::registry &);
void update_network_server(entt::registry &);

void server_handle_packet(entt::registry &, entt::entity client_entity, const packet::edyn_packet &);

void server_make_client(entt::registry &, entt::entity);
entt::entity server_make_client(entt::registry &);

/**
 * @brief Set client latency, which is often calculated as half the
 * round-trip time.
 * @param registry Data source.
 * @param client_entity The client to assign the latency to.
 * @param latency The latency in ms.
 */
void server_set_client_latency(entt::registry &, entt::entity client_entity, double latency);

/**
 * @brief Notify client of created entities. Usually called after the server
 * creates entities for the client. It's unnecessary otherwise, since the
 * server uses the AABB of interest to notify clients of entities that have
 * entered or left their AABB of interest.
 * @param registry Data source.
 * @param client_entity The client to be notified.
 * @param entities The new entities.
 */
void server_notify_created_entities(entt::registry &registry,
                                    entt::entity client_entity,
                                    const std::vector<entt::entity> &entities);

}

#endif // EDYN_NETWORKING_SERVER_SIDE_HPP
