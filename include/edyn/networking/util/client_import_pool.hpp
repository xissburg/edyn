#ifndef EDYN_NETWORKING_CLIENT_IMPORT_POOL_HPP
#define EDYN_NETWORKING_CLIENT_IMPORT_POOL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

template<typename Component>
void import_pool_client(entt::registry &registry, const std::vector<std::pair<entt::entity, Component>> &pool) {
    auto &ctx = registry.ctx<client_networking_context>();
    auto merge_ctx = merge_context{&registry, &ctx.entity_map};

    for (auto &pair : pool) {
        auto remote_entity = pair.first;

        if (!ctx.entity_map.has_rem(pair.first)) {
            // Entity not present in client. Send an entity request to server.
            ctx.request_entity_signal.publish(remote_entity);
            continue;
        }

        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (!registry.valid(local_entity)) {
            ctx.entity_map.erase_loc(local_entity);
            continue;
        }

        if constexpr(std::is_empty_v<Component>) {
            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);
                registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
            }
        } else {
            auto comp = pair.second;
            merge(static_cast<Component *>(nullptr), comp, merge_ctx);

            if (registry.any_of<Component>(local_entity)) {
                registry.replace<Component>(local_entity, comp);
                refresh<Component>(registry, local_entity);
            } else {
                registry.emplace<Component>(local_entity, comp);
                registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
            }
        }
    }
}

template<typename... Component>
void import_pool_client(entt::registry &registry, const pool_snapshot &pool, const std::tuple<Component...> &all_components) {
    visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
        using CompType = std::decay_t<decltype(c)>;
        import_pool_client(registry, std::static_pointer_cast<pool_snapshot_data<CompType>>(pool.ptr)->pairs);
    });
}

inline void import_pool_client_default(entt::registry &registry, const pool_snapshot &pool) {
    import_pool_client(registry, pool, networked_components);
}

}

#endif // EDYN_NETWORKING_CLIENT_IMPORT_POOL_HPP
