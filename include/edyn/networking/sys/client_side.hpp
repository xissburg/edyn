#ifndef EDYN_NETWORKING_CLIENT_SIDE_HPP
#define EDYN_NETWORKING_CLIENT_SIDE_HPP

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

void client_instantiate_entity_indices(entt::registry &registry, entt::entity entity,
                                       std::vector<component_index_type> &sync_indices);

template<typename... Component>
void client_instantiate_entity(entt::registry &registry, entt::entity entity)
{
    auto &ctx = registry.ctx().at<client_network_context>();
    auto indices = std::vector<component_index_type>{};
    indices.reserve(sizeof...(Component));
    (indices.push_back(ctx.snapshot_exporter->get_component_index<Component>()), ...);
    client_instantiate_entity_indices(registry, entity, indices);
}

}

#endif // EDYN_NETWORKING_CLIENT_SIDE_HPP
