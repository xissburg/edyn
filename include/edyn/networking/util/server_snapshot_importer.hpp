#ifndef EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/networking/comp/network_dirty.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/parallel/map_child_entity.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity);

class server_snapshot_importer {
public:
    virtual ~server_snapshot_importer() = default;

    // Import components in pool into registry. If the island where the entity
    // resides is not fully owned by the given client, the update won't be applied.
    // Input components of entities owned by the client are always applied.
    virtual void import(entt::registry &registry, entt::entity client_entity,
                        const registry_snapshot &snap, bool check_ownership, bool mark_dirty) = 0;

    // Import input components of a pool containing local entities.
    virtual void import_input_local(entt::registry &registry, entt::entity client_entity,
                                    const registry_snapshot &snap, bool mark_dirty) = 0;

    // Transform contained entities from remote to local using the remote client's entity map.
    virtual void transform_to_local(const entt::registry &registry, entt::entity client_entity,
                                    registry_snapshot &snap, bool check_ownership) = 0;
};

template<typename... Components>
class server_snapshot_importer_impl : public server_snapshot_importer {

    template<typename Component>
    bool is_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity local_entity) {
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
    void import_components(entt::registry &registry, entt::entity client_entity,
                           const std::vector<entt::entity> &pool_entities,
                           const pool_snapshot_data_impl<Component> &pool,
                           bool check_ownership, bool mark_dirty) {
        auto &client = registry.get<remote_client>(client_entity);

        for (size_t i = 0; i < pool.entity_indices.size(); ++i) {
            auto entity_index = pool.entity_indices[i];
            auto remote_entity = pool_entities[entity_index];

            if (!client.entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = client.entity_map.at(remote_entity);

            if (!registry.valid(local_entity)) {
                client.entity_map.erase(remote_entity);
                continue;
            }

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, local_entity)) {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);

                    if (mark_dirty) {
                        registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                    }
                }
            } else {
                auto comp = pool.components[i];
                internal::map_child_entity(registry, client.entity_map, comp);

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
    }

    template<typename Component>
    void import_input_components_local(entt::registry &registry, entt::entity client_entity,
                                       const std::vector<entt::entity> &pool_entities,
                                       const pool_snapshot_data_impl<Component> &pool,
                                       bool mark_dirty) {
        auto owner_view = registry.view<entity_owner>();

        for (size_t i = 0; i < pool.entity_indices.size(); ++i) {
            auto entity_index = pool.entity_indices[i];
            auto local_entity = pool_entities[entity_index];

            if (!registry.valid(local_entity)) {
                continue;
            }

            // Never replace inputs owned by client.
            if (!owner_view.contains(local_entity) ||
                std::get<0>(owner_view.get(local_entity)).client_entity != client_entity)
            {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);

                    if (mark_dirty) {
                        registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                    }
                }
            } else {
                auto &comp = pool.components[i];

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
    }

    template<typename Component>
    void transform_components_to_local(const entt::registry &registry, entt::entity client_entity,
                                       const std::vector<entt::entity> &pool_entities,
                                       pool_snapshot_data_impl<Component> &pool,
                                       bool check_ownership) {
        auto &client = registry.get<remote_client>(client_entity);

        auto remove = [&] (size_t i) {
            pool.entity_indices[i] = pool.entity_indices.back(), pool.entity_indices.pop_back();
            pool.components[i] = pool.components.back(), pool.components.pop_back();
        };

        // Do not increment `i` here because items will be removed by swapping
        // with last and popping.
        for (size_t i = 0; i < pool.entity_indices.size();) {
            auto entity_index = pool.entity_indices[i];
            auto entity = pool_entities[entity_index];

            if (!registry.valid(entity)) {
                remove(i);
                continue;
            }

            if (check_ownership && !is_owned_by_client<Component>(registry, client_entity, entity)) {
                remove(i);
                continue;
            }

            auto &comp = pool.components[i];
            internal::map_child_entity(registry, client.entity_map, comp);

            ++i;
        }
    }

public:
    template<typename... Input>
    server_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>,
                                  [[maybe_unused]] std::tuple<Input...>) {
        static_assert((!std::is_empty_v<Input> && ...));
        ((m_is_input_component[entt::type_id<Components>().seq()] = has_type<Components, std::tuple<Input...>>::value), ...);
    }

    void import(entt::registry &registry, entt::entity client_entity,
                const registry_snapshot &snap, bool check_ownership, bool mark_dirty) override {
        const std::tuple<Components...> all_components;

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                using CompType = std::decay_t<decltype(c)>;
                auto *typed_pool = static_cast<pool_snapshot_data_impl<CompType> *>(pool.ptr.get());
                import_components(registry, client_entity, snap.entities, *typed_pool, check_ownership, mark_dirty);
            });
        }
    }

    void import_input_local(entt::registry &registry, entt::entity client_entity,
                            const registry_snapshot &snap, bool mark_dirty) override {
        const std::tuple<Components...> all_components;

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                using CompType = std::decay_t<decltype(c)>;

                if (m_is_input_component.at(entt::type_id<CompType>().seq())) {
                    auto *typed_pool = static_cast<pool_snapshot_data_impl<CompType> *>(pool.ptr.get());
                    import_input_components_local(registry, client_entity, snap.entities, *typed_pool, mark_dirty);
                }
            });
        }
    }

    void transform_to_local(const entt::registry &registry, entt::entity client_entity,
                            registry_snapshot &snap, bool check_ownership) override {
        const std::tuple<Components...> all_components;

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                using CompType = std::decay_t<decltype(c)>;

                if constexpr(!std::is_empty_v<CompType>) {
                    auto *typed_pool = static_cast<pool_snapshot_data_impl<CompType> *>(pool.ptr.get());
                    transform_components_to_local(registry, client_entity, snap.entities, *typed_pool, check_ownership);
                }
            });
        }
    }

private:
    std::map<entt::id_type, bool> m_is_input_component;
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP
