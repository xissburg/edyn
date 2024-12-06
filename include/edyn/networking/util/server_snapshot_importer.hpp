#ifndef EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/comp/action_list.hpp"
#include "edyn/comp/child_list.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/merge_component.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/replication/map_child_entity.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

class server_snapshot_importer {
public:
    virtual ~server_snapshot_importer() = default;

    // Import components in pool into registry. If the island where the entity
    // resides is not fully owned by the given client, the update won't be applied.
    // Input components of entities owned by the client are always applied.
    virtual void import(entt::registry &registry, entt::entity client_entity,
                        const packet::registry_snapshot &snap, bool check_ownership) = 0;

    // Transform contained entities from remote to local using the remote
    // client's entity map.
    virtual void transform_to_local(const entt::registry &registry, entt::entity client_entity,
                                    packet::registry_snapshot &snap, bool check_ownership) = 0;

    // Merge all action_history components in the snapshot with corresponding
    // components in the registry.
    virtual void merge_action_history(entt::registry &registry, packet::registry_snapshot &snap,
                                      double time_delta) = 0;

    void import_action(entt::registry &registry, entt::entity entity,
                       action_history::action_index_type action_index,
                       const std::vector<uint8_t> &data) {
        if (m_import_action_func) {
            (*m_import_action_func)(registry, entity, action_index, data);
        }
    }

protected:
    using import_action_func_t = void(entt::registry &, entt::entity,
                                      action_history::action_index_type,
                                      const std::vector<uint8_t> &);
    import_action_func_t *m_import_action_func {nullptr};
};

template<typename... Components>
class server_snapshot_importer_impl : public server_snapshot_importer {

    enum class reach_status : uint8_t {
        unknown = 0,
        single_client,
        multiple_clients
    };

    bool is_only_reachable_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity) {
        // Traverse the graph starting at the given entity and check if
        // `client_entity` is the only client reachable from it.
        auto &graph = registry.ctx().get<entity_graph>();
        auto node_view = registry.view<graph_node>();
        auto edge_view = registry.view<graph_edge>();
        auto owner_view = registry.view<entity_owner>();
        auto child_view = registry.view<child_list>();

        entity_graph::index_type node_index;
        bool dest_client_reachable = false;
        bool other_client_reachable = false;

        if (edge_view.contains(entity)) {
            auto [edge] = edge_view.get(entity);
            node_index = graph.edge_node_indices(edge.edge_index)[0];
        } else if (node_view.contains(entity)) {
            auto [node] = node_view.get(entity);
            node_index = node.node_index;
        } else {
            // Must be a child of a node.
            auto [child] = child_view.get(entity);
            auto [node] = node_view.get(child.parent);
            node_index = node.node_index;
        }

        graph.traverse(node_index, [&](auto node_index) {
            auto neighbor = graph.node_entity(node_index);

            if (owner_view.contains(neighbor)) {
                auto [owner] = owner_view.get(neighbor);

                if (owner.client_entity == client_entity) {
                    dest_client_reachable = true;
                } else if (owner.client_entity != entt::null) {
                    other_client_reachable = true;
                }
            }
        });

        // The client temporarily owns the entity if it's the only client
        // reachable through the graph.
        return dest_client_reachable && !other_client_reachable;
    }

    template<typename Component>
    void import_components(entt::registry &registry, entt::entity client_entity,
                           const std::vector<entt::entity> &pool_entities,
                           const pool_snapshot_data_impl<Component> &pool,
                           bool check_ownership, std::vector<reach_status> &reach_cache) {
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

            if (check_ownership) {
                // If input or action history, ignore if the entity is not owned
                // by this client.
                if constexpr(std::is_base_of_v<network_input, Component> ||
                             std::is_same_v<action_history, Component>) {
                    if (auto *owner = registry.try_get<entity_owner>(local_entity);
                        !owner || owner->client_entity != client_entity)
                    {
                        continue;
                    }
                } else {
                    if (reach_cache[entity_index] == reach_status::unknown) {
                        reach_cache[entity_index] =
                            is_only_reachable_client(registry, client_entity, local_entity) ?
                                reach_status::single_client : reach_status::multiple_clients;
                    }

                    if (reach_cache[entity_index] == reach_status::multiple_clients) {
                        // Ignore if entity is in island with more than one client.
                        continue;
                    }
                }
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);
                }
            } else {
                auto comp = pool.components[i];
                internal::map_child_entity(registry, client.entity_map, comp);

                if (registry.any_of<Component>(local_entity)) {
                    registry.patch<Component>(local_entity, [&](auto &&current) {
                        merge_component(current, comp);
                    });
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
                                       bool check_ownership, std::vector<reach_status> &reach_cache) {
        auto &client = registry.get<remote_client>(client_entity);

        auto remove = [&](size_t i) {
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

            if (check_ownership) {
                // If input or action history, ignore if the entity is not owned
                // by this client.
                if constexpr(std::is_base_of_v<network_input, Component> ||
                             std::is_same_v<action_history, Component>) {
                    if (auto *owner = registry.try_get<entity_owner>(entity);
                        !owner || owner->client_entity != client_entity)
                    {
                        remove(i);
                        continue;
                    }
                } else {
                    if (reach_cache[entity_index] == reach_status::unknown) {
                        reach_cache[entity_index] =
                            is_only_reachable_client(registry, client_entity, entity) ?
                                reach_status::single_client : reach_status::multiple_clients;
                    }

                    if (reach_cache[entity_index] == reach_status::multiple_clients) {
                        // Ignore if entity is in island with more than one client.
                        remove(i);
                        continue;
                    }
                }
            }

            auto &comp = pool.components[i];
            internal::map_child_entity(registry, client.entity_map, comp);

            ++i;
        }
    }

    template<typename Action>
    static void import_action_single(entt::registry &registry, entt::entity entity,
                                     const std::vector<uint8_t> &data) {
        using ActionListType = action_list<Action>;
        ActionListType import_list;
        auto archive = memory_input_archive(data.data(), data.size());
        archive(import_list);

        if (archive.failed()) {
            return;
        }

        if (!registry.all_of<ActionListType>(entity)) {
            registry.emplace<ActionListType>(entity);
        }

        registry.patch<ActionListType>(entity, [&import_list](auto &list) {
            list.actions.insert(list.actions.end(), import_list.actions.begin(), import_list.actions.end());
        });
    }

    template<typename... Actions>
    static auto import_action(entt::registry &registry, entt::entity entity,
                              action_history::action_index_type action_index,
                              const std::vector<uint8_t> &data) {
        static_assert(sizeof...(Actions) > 0);
        if constexpr(sizeof...(Actions) == 1) {
            (import_action_single<Actions>(registry, entity, data), ...);
        } else {
            static const auto actions = std::tuple<Actions...>{};
            visit_tuple(actions, action_index, [&](auto &&a) {
                using ActionType = std::decay_t<decltype(a)>;
                import_action_single<ActionType>(registry, entity, data);
            });
        }
    }

public:
    template<typename... Actions>
    server_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>,
                                  [[maybe_unused]] std::tuple<Actions...>) {
        if constexpr(sizeof...(Actions) > 0) {
            m_import_action_func = &import_action<Actions...>;
        }
    }

    void import(entt::registry &registry, entt::entity client_entity,
                const packet::registry_snapshot &snap, bool check_ownership) override {
        const std::tuple<Components...> all_components;
        std::vector<reach_status> reach_cache;

        if (check_ownership) {
            reach_cache.assign(snap.entities.size(), reach_status::unknown);
        }

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&](auto &&c) {
                using CompType = std::decay_t<decltype(c)>;
                auto *typed_pool = static_cast<pool_snapshot_data_impl<CompType> *>(pool.ptr.get());
                import_components(registry, client_entity, snap.entities, *typed_pool, check_ownership, reach_cache);
            });
        }
    }

    void transform_to_local(const entt::registry &registry, entt::entity client_entity,
                            packet::registry_snapshot &snap, bool check_ownership) override {
        const std::tuple<Components...> all_components;
        std::vector<reach_status> reach_cache;

        if (check_ownership) {
            reach_cache.assign(snap.entities.size(), reach_status::unknown);
        }

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&](auto &&c) {
                using CompType = std::decay_t<decltype(c)>;

                if constexpr(!std::is_empty_v<CompType>) {
                    auto *typed_pool = static_cast<pool_snapshot_data_impl<CompType> *>(pool.ptr.get());
                    transform_components_to_local(registry, client_entity, snap.entities, *typed_pool, check_ownership, reach_cache);
                }
            });
        }
    }

    void merge_action_history(entt::registry &registry, packet::registry_snapshot &snap, double time_delta) override {
        auto pool_it = std::find_if(snap.pools.begin(), snap.pools.end(), [](auto &&pool) {
            auto action_history_index = index_of_v<component_index_type, action_history, Components...>;
            return pool.component_index == action_history_index;
        });

        if (pool_it == snap.pools.end()) {
            return;
        }

        auto *history_pool = static_cast<pool_snapshot_data_impl<action_history> *>(pool_it->ptr.get());

        for (size_t i = 0; i < history_pool->components.size(); ++i) {
            auto idx = history_pool->entity_indices[i];
            auto entity = snap.entities[idx];
            auto &history = history_pool->components[i];

            if (history.empty()) {
                continue;
            }

            history.sort();

            for (auto &entry : history.entries) {
                entry.timestamp += time_delta;
            }

            registry.patch<action_history>(entity, [&](action_history &current) {
                current.merge(history);
            });
        }

        *pool_it = std::move(snap.pools.back());
        snap.pools.pop_back();
    }
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_IMPORTER_HPP
