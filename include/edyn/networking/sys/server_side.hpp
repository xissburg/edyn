#ifndef EDYN_NETWORKING_SERVER_SIDE_HPP
#define EDYN_NETWORKING_SERVER_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

/**
 * @brief Initialize a network server context into the given registry. Edyn
 * must have been initialized and attached to the same registry prior to this
 * call.
 * @param registry Data source.
 */
void init_network_server(entt::registry &);

/**
 * @brief Remove network server context from registry where it was previously
 * initialized.
 * @param registry Data source.
 */
void deinit_network_server(entt::registry &);

/**
 * @brief Must be called frequently to run the network server duties. This
 * function will process received packets and generate packets to be sent to
 * clients.
 * @param registry Data source.
 */
void update_network_server(entt::registry &);

/**
 * @brief Receives an Edyn packet from a client. Must be called for every
 * packet received.
 * @param registry Data source.
 * @param client_entity Client who sent the packet.
 * @param packet Incoming packet.
 */
void server_receive_packet(entt::registry &, entt::entity client_entity, const packet::edyn_packet &);

/**
 * @brief Create a new client. Must be called when a connection is established
 * with a new client.
 * @param registry Data source.
 * @param client_entity The entity to be used to identify the client.
 */
void server_make_client(entt::registry &, entt::entity);

/**
 * @brief Create a new client. Must be called when a connection is established
 * with a new client.
 * @param registry Data source.
 * @return The created client entity which will be used to identify the client.
 */
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
