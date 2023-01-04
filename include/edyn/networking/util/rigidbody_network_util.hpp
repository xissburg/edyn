#ifndef EDYN_NETWORKING_UTIL_RIGIDBODY_NETWORK_UTIL_HPP
#define EDYN_NETWORKING_UTIL_RIGIDBODY_NETWORK_UTIL_HPP

#include "edyn/networking/comp/asset_entry.hpp"
#include "edyn/networking/comp/asset_ref.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/util/rigidbody.hpp"

namespace edyn {

template<typename... Component>
void set_rigidbody_networked(rigidbody_def &def, const entt::registry &registry, entt::entity asset_entity, entt::id_type id) {
    auto entry = asset_entry{};
    entry.asset_entity = asset_entity;
    entry.id = id;
    EDYN_ASSERT(registry.all_of<asset_ref>(asset_entity));

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        (entry.sync_indices.push_back(ctx->snapshot_exporter->get_component_index<Component>()), ...);
        def.network_info = std::move(entry);
    } else if (auto *ctx = registry.ctx().find<server_network_context>()) {
        (entry.sync_indices.push_back(ctx->snapshot_exporter->get_component_index<Component>()), ...);
        def.network_info = std::move(entry);
    }
}

}

#endif // EDYN_NETWORKING_UTIL_RIGIDBODY_NETWORK_UTIL_HPP
