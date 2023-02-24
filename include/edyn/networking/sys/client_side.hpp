#ifndef EDYN_NETWORKING_CLIENT_SIDE_HPP
#define EDYN_NETWORKING_CLIENT_SIDE_HPP

#include <map>
#include <unordered_map>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/util/component_index_type.hpp"

namespace edyn {

/**
 * @brief Initialize a network client context into the given registry. Edyn
 * must have been initialized and attached to the same registry prior to this
 * call.
 * @param registry Data source.
 */
void init_network_client(entt::registry &);

/**
 * @brief Remove network client context from registry where it was previously
 * initialized.
 * @param registry Data source.
 */
void deinit_network_client(entt::registry &);

/**
 * @brief Must be called frequently to run the network client logic. This
 * function will process received packets and generate packets to be sent to
 * server.
 * @param registry Data source.
 */
void update_network_client(entt::registry &);

/**
 * @brief Receives an Edyn packet from server. Must be called for every packet
 * received.
 * @param registry Data source.
 * @param packet Incoming packet.
 */
void client_receive_packet(entt::registry &, packet::edyn_packet &);

/**
 * @brief Check whether the current client owns the given networked entity.
 * @param registry Data source.
 * @param entity Networked entity.
 * @return Whether the networked entity is owned by the current client.
 */
bool client_owns_entity(const entt::registry &, entt::entity);

/**
 * @brief Notify Edyn that asset is ready to instantiate entities.
 * @remark The server will be asked to send a snapshot containing relevant
 * initial state to be applied immediately after local instantiation.
 * When the snapshot is received, the `edyn::network_client_instantiate_asset_sink`
 * signal will be triggered.
 * @param registry Data source.
 * @param entity Asset entity.
 */
void client_asset_ready(entt::registry &registry, entt::entity entity);

/**
 * @brief Must be called after instantiating an entity from an asset to link
 * local entities to remote entities via the common internal asset ids.
 * @param registry Data source.
 * @param entity Asset entity.
 * @param emap Maps internal asset ids to local entities.
 */
void client_link_asset(entt::registry &registry, entt::entity entity,
                       const std::map<entt::id_type, entt::entity> &emap);

/*! @copydoc client_link_asset */
void client_link_asset(entt::registry &registry, entt::entity entity,
                       const std::unordered_map<entt::id_type, entt::entity> &emap);

}

#endif // EDYN_NETWORKING_CLIENT_SIDE_HPP
