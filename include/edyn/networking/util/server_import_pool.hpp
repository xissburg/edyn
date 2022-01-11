#ifndef EDYN_NETWORKING_SERVER_IMPORT_POOL_HPP
#define EDYN_NETWORKING_SERVER_IMPORT_POOL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/edyn.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/parallel/merge/merge_component.hpp"

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &, entt::entity client_entity, entt::entity entity);

template<typename Component>
void import_pool_server(entt::registry &registry, entt::entity client_entity,
                        const std::vector<std::pair<entt::entity, Component>> &pool, bool broadcast) {
    auto &client = registry.get<remote_client>(client_entity);
    auto &settings = registry.ctx<edyn::settings>();
    auto comp_index = settings.index_source->index_of<Component>();

    for (auto &pair : pool) {
        auto remote_entity = pair.first;

        if (!client.entity_map.has_rem(remote_entity)) {
            continue;
        }

        auto local_entity = client.entity_map.remloc(remote_entity);

        if (!registry.valid(local_entity)) {
            client.entity_map.erase_loc(local_entity);
            continue;
        }

        // If this is a procedural component and the entity is not fully owned
        // by the client, the update must not be applied, because in this case
        // the server is in control of the procedural state.
        auto *comp_list = registry.try_get<non_proc_comp_list>(local_entity);
        auto is_procedural = !comp_list || !comp_list->contains(comp_index);

        if (is_procedural && !is_fully_owned_by_client(registry, client_entity, local_entity)) {
            continue;
        }

        if constexpr(std::is_empty_v<Component>) {
            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);
                registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
            }
        } else {
            auto comp = pair.second;
            merge(comp, client.entity_map);

            if (registry.any_of<Component>(local_entity)) {
                registry.replace<Component>(local_entity, comp);
                refresh<Component>(registry, local_entity);

                if (broadcast) {
                    // Broadcast change to other clients which contain this entity in
                    // their AABB of interest.
                    registry.view<remote_client>().each([&] (entt::entity other_client_entity, remote_client &client) {
                        if (other_client_entity == client_entity) return;
                        insert_entity_component<Component>(registry, local_entity, client.current_snapshot.pools);
                    });
                }
            } else {
                registry.emplace<Component>(local_entity, comp);
                registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
            }
        }
    }
}

template<typename... Component>
void import_pool_server(entt::registry &registry, entt::entity client_entity, const pool_snapshot &pool, bool broadcast, const std::tuple<Component...> &all_components) {
    visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
        using CompType = std::decay_t<decltype(c)>;
        import_pool_server(registry, client_entity, std::static_pointer_cast<pool_snapshot_data<CompType>>(pool.ptr)->pairs, broadcast);
    });
}

inline void import_pool_server_default(entt::registry &registry, entt::entity client_entity, const pool_snapshot &pool, bool broadcast) {
    import_pool_server(registry, client_entity, pool, broadcast, networked_components);
}

}

#endif // EDYN_NETWORKING_SERVER_IMPORT_POOL_HPP
