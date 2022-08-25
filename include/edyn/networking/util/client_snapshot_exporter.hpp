#ifndef EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/fwd.hpp>
#include <type_traits>
#include "edyn/comp/action_list.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

class client_snapshot_exporter {
public:
    virtual ~client_snapshot_exporter() = default;

    // Write all networked entities and components into a snapshot.
    virtual void export_all(const entt::registry &registry, packet::registry_snapshot &snap) = 0;

    // Write all modified networked entities and components into a snapshot.
    virtual void export_modified(const entt::registry &registry, packet::registry_snapshot &snap) = 0;

    void export_actions(const entt::registry &registry, packet::registry_snapshot &snap) {
        internal::snapshot_insert_all<action_history>(registry, snap,
            tuple_index_of<unsigned, action_history>(networked_components));
    }

    void append_current_actions(entt::registry &registry, double time) {
        if (m_append_current_actions_func != nullptr) {
            (*m_append_current_actions_func)(registry, time);
        }
    }

    virtual void update(double time) = 0;

protected:
    using append_current_actions_func_t = void(entt::registry &registry, double time);
    append_current_actions_func_t *m_append_current_actions_func {nullptr};
};

template<typename... Components>
class client_snapshot_exporter_impl : public client_snapshot_exporter {

    struct modified_components {
        std::array<unsigned short, sizeof...(Components)> time_remaining {};

        bool empty() const {
            return std::accumulate(time_remaining.begin(), time_remaining.end(), 0) > 0;
        }
    };

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        static constexpr auto index = index_of_v<unsigned, Component, Components...>;
        registry.get<modified_components>(entity).time_remaining[index] = 400;
    }

    template<typename Action>
    static void append_actions(entt::registry &registry, double time) {
        auto view = registry.view<action_list<Action>, action_history>();

        for (auto [entity, list, history] : view.each()) {
            if (list.actions.empty()) {
                continue;
            }

            auto data = std::vector<uint8_t>{};
            auto archive = memory_output_archive(data);
            archive(list);
            history.entries.emplace_back(time, std::move(data));
        }
    }

    template<typename... Actions>
    static void append_current_actions(entt::registry &registry, double time) {
        (append_actions<Actions>(registry, time), ...);
    }

public:
    template<typename... Actions>
    client_snapshot_exporter_impl([[maybe_unused]] std::tuple<Components...>,
                                  [[maybe_unused]] std::tuple<Actions...>) {
        if constexpr(sizeof...(Actions) > 0) {
            m_append_current_actions_func = &append_current_actions<Actions...>;
        }
    }

    void export_all(const entt::registry &registry, packet::registry_snapshot &snap) override {
        const std::tuple<Components...> components;
        internal::snapshot_insert_entity_components_all(registry, snap, components,
                                                        std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_modified(const entt::registry &registry, packet::registry_snapshot &snap,
                         entt::entity client_entity, const entt::sparse_set &owned_entities,
                         bool allow_full_ownership) override {
        auto modified_view = registry.view<modified_components>();

        if (allow_full_ownership) {
            // Include all networked entities in the islands that contain an entity
            // owned by this client, excluding entities that are owned by other clients.
            auto island_entities = collect_islands_from_residents(registry, owned_entities.begin(), owned_entities.end());
            auto owner_view = registry.view<entity_owner>();
            auto island_view = registry.view<island>();

            for (auto island_entity : island_entities) {
                auto [island] = island_view.get(island_entity);

                for (auto entity : island.nodes) {
                    if (modified_view.contains(entity)) {
                        auto is_owned_by_another_client =
                            owner_view.contains(entity) &&
                            std::get<0>(owner_view.get(entity)).client_entity != client_entity;

                        if (!is_owned_by_another_client && !std::get<0>(modified_view.get(entity)).empty()) {
                            snap.entities.push_back(entity);
                        }
                    }
                }
            }
        } else {
            // Otherwise, only entities owned by this client which contain an
            // input component are included.
            for (auto entity : owned_entities) {
                auto [modified] = modified_view.get(entity);
                unsigned i = 0;
                ((std::is_base_of_v<network_input, Components> && registry.all_of<Components>(entity) && modified.time_remaining[i++] > 0 ?
                    snap.entities.push_back(entity) : void(0)), ...);
            }

            for (auto entity : owned_entities) {
                auto [modified] = modified_view.get(entity);
                unsigned i = 0;
                ((std::is_base_of_v<network_input, Components> && registry.all_of<Components>(entity) && modified.time_remaining[i++] > 0 ?
                    internal::get_pool<Components>(snap.pools, i)->insert_single(registry, entity, snap.entities) : void(0)), ...);
            }
        }

        auto history_view = registry.view<action_history>();

        for (auto entity : owned_entities) {
            if (history_view.contains(entity) && !std::get<0>(history_view.get(entity)).entries.empty()) {
                snap.entities.push_back(entity);
            }
        }

        constexpr auto indices = std::make_integer_sequence<unsigned, sizeof...(Components)>{};
        auto network_dirty_view = registry.view<network_dirty>();

        network_dirty_view.each([&](entt::entity entity, const network_dirty &n_dirty) {
            n_dirty.each([&](entt::id_type id) {
                export_by_type_id(registry, entity, id, snap, indices);
            });
        });

        // Always include actions.
        export_actions(registry, packet);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP
