#ifndef EDYN_NETWORKING_UTIL_ASSET_UTIL_HPP
#define EDYN_NETWORKING_UTIL_ASSET_UTIL_HPP

#include "edyn/networking/comp/asset_entry.hpp"
#include "edyn/networking/comp/asset_ref.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/util/rigidbody.hpp"

namespace edyn {

template<typename... Component>
void assign_to_asset(entt::registry &registry, entt::entity entity, entt::entity asset_entity, entt::id_type id) {
    registry.get<asset_ref>(asset_entity).entity_map[id] = entity;
    registry.emplace<networked_tag>(entity);

    auto &entry = registry.emplace<asset_entry>(entity);
    entry.id = id;
    entry.asset_entity = asset_entity;

    if constexpr(sizeof...(Component) > 0) {
        if (auto *ctx = registry.ctx().find<client_network_context>()) {
            (entry.sync_indices.push_back(ctx->snapshot_exporter->get_component_index<Component>()), ...);
        } else if (auto *ctx = registry.ctx().find<server_network_context>()) {
            (entry.sync_indices.push_back(ctx->snapshot_exporter->get_component_index<Component>()), ...);
        }
    }
}

}

#endif // EDYN_NETWORKING_UTIL_ASSET_UTIL_HPP
