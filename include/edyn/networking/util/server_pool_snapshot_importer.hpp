#ifndef EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/comp/network_dirty.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity);

class server_pool_snapshot_importer {
public:
    // Import components in pool into registry. If the island where the entity
    // resides is not fully owned by the given client, the update won't be applied.
    // Input components of entities owned by the client are always applied.
    virtual void import(entt::registry &registry, entt::entity client_entity,
                        const pool_snapshot &pool, bool check_ownership, bool mark_dirty) = 0;

    // Import input components of a pool containing local entities.
    virtual void import_input_local(entt::registry &registry, entt::entity client_entity,
                                    const pool_snapshot &pool, bool mark_dirty) = 0;

    // Transform entities from remote to local using the remote client's entity map.
    virtual void transform_to_local(entt::registry &registry, entt::entity client_entity,
                                    pool_snapshot &pool, bool check_ownership) = 0;
};

template<typename... Components>
class server_pool_snapshot_importer_impl : public server_pool_snapshot_importer {

    template<typename Component>
    bool is_owned_by_client(entt::registry &registry, entt::entity client_entity, entt::entity local_entity) {
        // If the entity is not fully owned by the client, the update must
        // not be applied, because in this case the server is in control of
        // the procedural state. Input components are one exception because
        // they must always be applied.
        auto is_input = m_is_input_component.at(entt::type_id<Component>().seq());

        if (is_input) {
            if (auto *owner = registry.try_get<entity_owner>(local_entity);
                owner && owner->client_entity == client_entity)
            {
                return true;
            }
        } else if (is_fully_owned_by_client(registry, client_entity, local_entity)) {
            return true;
        }

        return false;
    }

    template<typename Component>
    void import_pairs(entt::registry &registry, entt::entity client_entity,
                      const std::vector<std::pair<entt::entity, Component>> &pairs,
                      bool check_ownership, bool mark_dirty) {
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

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, local_entity)) {
                continue;
            }

            auto comp = pair.second;
            merge(comp, client.entity_map);

            if (mark_dirty) {
                auto &dirty = registry.get_or_emplace<network_dirty>(local_entity);

                if (registry.any_of<Component>(local_entity)) {
                    dirty.template updated<Component>();
                } else {
                    dirty.template created<Component>();
                }
            }

            if (registry.any_of<Component>(local_entity)) {
                registry.replace<Component>(local_entity, comp);
            } else {
                registry.emplace<Component>(local_entity, comp);
            }
        }
    }

    template<typename Component>
    void import_entities(entt::registry &registry, entt::entity client_entity,
                         const std::vector<entt::entity> &entities,
                         bool check_ownership, bool mark_dirty) {
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

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, local_entity)) {
                continue;
            }

            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);

                if (mark_dirty) {
                    registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                }
            }
        }
    }

    template<typename Component>
    void import_input_pairs_local(entt::registry &registry, entt::entity client_entity,
                                  const std::vector<std::pair<entt::entity, Component>> &pairs,
                                  bool mark_dirty) {
        auto owner_view = registry.view<entity_owner>();

        for (auto &pair : pairs) {
            auto local_entity = pair.first;

            if (!owner_view.contains(local_entity) ||
                std::get<0>(owner_view.get(local_entity)).client_entity != client_entity)
            {
                continue;
            }

            auto &comp = pair.second;

            if (mark_dirty) {
                auto &dirty = registry.get_or_emplace<network_dirty>(local_entity);

                if (registry.any_of<Component>(local_entity)) {
                    dirty.template updated<Component>();
                } else {
                    dirty.template created<Component>();
                }
            }

            if (registry.any_of<Component>(local_entity)) {
                registry.replace<Component>(local_entity, comp);
            } else {
                registry.emplace<Component>(local_entity, comp);
            }
        }
    }

    template<typename Component>
    void import_input_entities_local(entt::registry &registry, entt::entity client_entity,
                                     const std::vector<entt::entity> &entities,
                                     bool mark_dirty) {
        auto owner_view = registry.view<entity_owner>();

        for (auto local_entity : entities) {
            if (!owner_view.contains(local_entity) ||
                std::get<0>(owner_view.get(local_entity)).client_entity != client_entity)
            {
                continue;
            }

            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);

                if (mark_dirty) {
                    registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                }
            }
        }
    }

    template<typename Component>
    void transform_pairs_to_local(entt::registry &registry, entt::entity client_entity,
                                  std::vector<std::pair<entt::entity, Component>> &pairs,
                                  bool check_ownership) {
        auto &client = registry.get<remote_client>(client_entity);

        auto assign_value_of_last_and_pop_back = [&pairs] (auto &it) {
            *it = pairs.back();
            pairs.pop_back();
        };

        for (auto it = pairs.begin(); it != pairs.end();) {
            auto &pair = *it;
            auto &remote_entity = pair.first;

            if (!client.entity_map.has_rem(remote_entity)) {
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            auto local_entity = client.entity_map.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                client.entity_map.erase_loc(local_entity);
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, local_entity)) {
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            remote_entity = local_entity;
            auto &comp = pair.second;
            merge(comp, client.entity_map);
            ++it;
        }
    }

    template<typename Component>
    void transform_entities_to_local(entt::registry &registry, entt::entity client_entity,
                                     std::vector<entt::entity> &entities, bool check_ownership) {
        auto &client = registry.get<remote_client>(client_entity);

        auto assign_value_of_last_and_pop_back = [&entities] (auto &it) {
            *it = entities.back();
            entities.pop_back();
        };

        for (auto it = entities.begin(); it != entities.end();) {
            auto &remote_entity = *it;

            if (!client.entity_map.has_rem(remote_entity)) {
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            auto local_entity = client.entity_map.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                client.entity_map.erase_loc(local_entity);
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, local_entity)) {
                assign_value_of_last_and_pop_back(it);
                continue;
            }

            remote_entity = local_entity;
            ++it;
        }
    }

public:
    template<typename... Input>
    server_pool_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>,
                                       [[maybe_unused]] std::tuple<Input...>) {
        static_assert((!std::is_empty_v<Input> && ...));
        ((m_is_input_component[entt::type_id<Components>().seq()] = has_type<Components, std::tuple<Input...>>::value), ...);
    }

    void import(entt::registry &registry, entt::entity client_entity,
                const pool_snapshot &pool, bool check_ownership, bool mark_dirty) override {
        const std::tuple<Components...> all_components;

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<CompType>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<CompType>) {
                import_entities<CompType>(registry, client_entity, data, check_ownership, mark_dirty);
            } else {
                import_pairs<CompType>(registry, client_entity, data, check_ownership, mark_dirty);
            }
        });
    }

    void import_input_local(entt::registry &registry, entt::entity client_entity,
                            const pool_snapshot &pool, bool mark_dirty) override {
        const std::tuple<Components...> all_components;

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;

            if (!m_is_input_component.at(entt::type_id<CompType>().seq())) {
                return;
            }

            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<CompType>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<CompType>) {
                import_input_entities_local<CompType>(registry, client_entity, data, mark_dirty);
            } else {
                import_input_pairs_local<CompType>(registry, client_entity, data, mark_dirty);
            }
        });
    }

    void transform_to_local(entt::registry &registry, entt::entity client_entity,
                            pool_snapshot &pool, bool check_ownership) override {
        const std::tuple<Components...> all_components;

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<CompType>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<CompType>) {
                transform_entities_to_local<CompType>(registry, client_entity, data, check_ownership);
            } else {
                transform_pairs_to_local<CompType>(registry, client_entity, data, check_ownership);
            }
        });
    }

private:
    std::map<entt::id_type, bool> m_is_input_component;
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_IMPORTER_HPP
