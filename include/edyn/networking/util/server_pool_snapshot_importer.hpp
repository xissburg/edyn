#ifndef EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/comp/dirty.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity);

class server_pool_snapshot_importer {
public:
    virtual void import(entt::registry &registry, entt::entity client_entity, const pool_snapshot &pool, bool check_ownership) = 0;
};

template<typename... Components>
class server_pool_snapshot_importer_impl : public server_pool_snapshot_importer {

    template<typename Component>
    void import_pairs(entt::registry &registry, entt::entity client_entity,
                      const std::vector<std::pair<entt::entity, Component>> &pairs,
                      bool check_ownership) {
        auto &client = registry.get<remote_client>(client_entity);

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!client.entity_map.has_rem(remote_entity)) {
                continue;
            }

            auto local_entity = client.entity_map.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                client.entity_map.erase_loc(local_entity);
                continue;
            }

            if (check_ownership) {
                // If this is a procedural component and the entity is not fully owned
                // by the client, the update must not be applied, because in this case
                // the server is in control of the procedural state.
                auto is_procedural = m_is_procedural_component[entt::type_id<Component>().seq()];

                if (is_procedural && !is_fully_owned_by_client(registry, client_entity, local_entity)) {
                    continue;
                }
            }

            auto comp = pair.second;
            merge(comp, client.entity_map);
            auto &dirty = registry.get_or_emplace<edyn::dirty>(local_entity);

            if (registry.any_of<Component>(local_entity)) {
                registry.replace<Component>(local_entity, comp);
                dirty.template updated<Component>();
            } else {
                registry.emplace<Component>(local_entity, comp);
                dirty.template created<Component>();
            }
        }
    }

    template<typename Component>
    void import_entities(entt::registry &registry, entt::entity client_entity,
                         const std::vector<entt::entity> &entities,
                         bool check_ownership) {
        auto &client = registry.get<remote_client>(client_entity);

        for (auto remote_entity : entities) {
            if (!client.entity_map.has_rem(remote_entity)) {
                continue;
            }

            auto local_entity = client.entity_map.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                client.entity_map.erase_loc(local_entity);
                continue;
            }

            if (check_ownership) {
                // If this is a procedural component and the entity is not fully owned
                // by the client, the update must not be applied, because in this case
                // the server is in control of the procedural state.
                auto is_procedural = m_is_procedural_component[entt::type_id<Component>().seq()];

                if (is_procedural && !is_fully_owned_by_client(registry, client_entity, local_entity)) {
                    continue;
                }
            }

            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);
                registry.get_or_emplace<dirty>(local_entity).template created<Component>();
            }
        }
    }

public:
    template<typename... NonProcedural>
    server_pool_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>,
                                       [[maybe_unused]] std::tuple<NonProcedural...>) {
        static_assert((!std::is_empty_v<NonProcedural> && ...));

        ((m_is_procedural_component[entt::type_id<Components>().seq()] = !has_type<Components, std::tuple<NonProcedural...>>::value), ...);
    }

    void import(entt::registry &registry, entt::entity client_entity, const pool_snapshot &pool, bool check_ownership) override {
        const std::tuple<Components...> all_components;

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<CompType>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<CompType>) {
                import_entities<CompType>(registry, client_entity, data, check_ownership);
            } else {
                import_pairs<CompType>(registry, client_entity, data, check_ownership);
            }
        });
    }

private:
    std::map<entt::id_type, bool> m_is_procedural_component;
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP
