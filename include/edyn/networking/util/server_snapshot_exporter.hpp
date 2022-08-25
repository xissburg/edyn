#ifndef EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP

#include <entt/core/type_info.hpp>
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <entt/signal/sigh.hpp>
#include <numeric>
#include <type_traits>
#include <vector>
#include "edyn/comp/action_list.hpp"
#include "edyn/config/config.h"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/util/is_fully_owned_by_client.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

extern bool(*g_is_networked_input_component)(entt::id_type);
extern bool(*g_is_action_list_component)(entt::id_type);

class server_snapshot_exporter {
public:
    virtual ~server_snapshot_exporter() = default;

    // Write all networked entities and components into a snapshot.
    virtual void export_all(packet::registry_snapshot &snap, const entt::sparse_set &entities) const = 0;
    virtual void export_all(packet::registry_snapshot &snap, const std::vector<entt::entity> &entities) const = 0;

    // Write all components that have been recently modified into a snapshot.
    virtual void export_modified(packet::registry_snapshot &snap,
                                 const entt::sparse_set &entities_of_interest,
                                 entt::entity dest_client_entity) const = 0;

    // Decays the time remaining in each of the recently modified components.
    // They stop being included in the snapshot once the timer reaches zero.
    virtual void update(double time) = 0;
};

template<typename... Components>
class server_snapshot_exporter_impl : public server_snapshot_exporter {

    struct modified_components {
        std::array<unsigned short, sizeof...(Components)> time_remaining {};

        bool empty() const {
            return std::accumulate(time_remaining.begin(), time_remaining.end(), 0) > 0;
        }
    };

    template<typename Component, typename... Actions>
    constexpr bool is_action_list() {
        return std::disjunction_v<std::is_same<action_list<Actions>, Component>...>;
    }

public:
    template<typename... Actions>
    server_snapshot_exporter_impl(entt::registry &registry,
                                  [[maybe_unused]] std::tuple<Components...>,
                                  [[maybe_unused]] std::tuple<Actions...>)
        : m_registry(&registry)
    {
        m_connections.push_back(registry.on_construct<networked_tag>().connect<&entt::registry::emplace<modified_components>>());
        ((m_connections.push_back(registry.on_update<Components>().template connect<&server_snapshot_exporter_impl<Components...>::template on_update<Components>>(*this))), ...);

        unsigned i = 0;
        ((m_is_network_input[i++] = std::is_base_of_v<network_input, Components>), ...);

        i = 0;
        ((m_is_action_list[i++] = is_action_list<Components, Actions...>()), ...);
    }

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        static constexpr auto index = index_of_v<unsigned, Component, Components...>;
        registry.get<modified_components>(entity).time_remaining[index] = 400;
    }

    template<typename It>
    void export_all(packet::registry_snapshot &snap, It first, It last) const {
        for (; first != last; ++first) {
            auto entity = *first;
            unsigned i = 0;
            (((m_registry->all_of<Components>(entity) ?
                internal::snapshot_insert_entity<Components>(*m_registry, entity, snap, i) : void(0)), ++i), ...);
        }
    }

    void export_all(packet::registry_snapshot &snap, const std::vector<entt::entity> &entities) const override {
        export_all(snap, entities.begin(), entities.end());
    }

    void export_all(packet::registry_snapshot &snap, const entt::sparse_set &entities) const override {
        export_all(snap, entities.begin(), entities.end());
    }

    void export_modified(packet::registry_snapshot &snap,
                         const entt::sparse_set &entities_of_interest,
                         entt::entity dest_client_entity) const override {
        auto &registry = *m_registry;
        auto owner_view = registry.view<entity_owner>();
        auto modified_view = registry.view<modified_components>();

        // Collect all entities first.
        // Only include entities which are in islands not fully owned by the client
        // since the server allows the client to have full control over entities in
        // the islands where there are no other clients present.
        for (auto entity : entities_of_interest) {
            auto [modified] = modified_view.get(entity);

            if (!modified.empty() && !is_fully_owned_by_client(registry, dest_client_entity, entity)) {
                snap.entities.push_back(entity);
            }
        }

        // Export components.
        // Do not include input components of entities owned by destination
        // client as to not override client input on the client-side.
        // Clients own their input.
        for (auto entity : snap.entities) {
            auto owned_by_client = !owner_view.contains(entity) ? false :
                std::get<0>(owner_view.get(entity)).client_entity == dest_client_entity;
            auto [modified] = modified_view.get(entity);
            unsigned i = 0;
            (((modified.time_remaining[i] > 0 && (!owned_by_client || !(m_is_network_input[i] || m_is_action_list[i])) ?
                internal::get_pool<Components>(snap.pools, i)->insert_single(registry, entity, snap.entities) : void(0)), ++i), ...);
        }
    }

    void update(double time) override {
        EDYN_ASSERT(!(time < m_last_time));
        auto elapsed_ms = static_cast<unsigned>(time - m_last_time) * 1000u;
        m_last_time = time;

        m_registry->view<modified_components>().each([&](modified_components &modified) {
            for (auto &remaining : modified.time_remaining) {
                if (elapsed_ms > remaining) {
                    remaining = 0;
                } else {
                    remaining -= elapsed_ms;
                }
            }
        });
    }

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    std::array<bool, sizeof...(Components)> m_is_network_input;
    std::array<bool, sizeof...(Components)> m_is_action_list;
    double m_last_time {};
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
