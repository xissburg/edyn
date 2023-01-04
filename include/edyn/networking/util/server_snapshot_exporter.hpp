#ifndef EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/registry.hpp>
#include <entt/signal/sigh.hpp>
#include <numeric>
#include <type_traits>
#include <vector>
#include "edyn/comp/action_list.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include "edyn/core/entity_graph.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/util/component_index_type.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

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

    virtual void export_comp_index(packet::registry_snapshot &snap, entt::entity entity,
                                   const std::vector<component_index_type> &indices) const = 0;

    // Decays the time remaining in each of the recently modified components.
    // They stop being included in the snapshot once the timer reaches zero.
    virtual void update(double time) = 0;

    template<typename Component>
    component_index_type get_component_index() const {
        auto id = entt::type_index<Component>();
        return m_component_indices.at(id);
    }

protected:
    std::map<entt::id_type, component_index_type> m_component_indices;
};

template<typename... Components>
class server_snapshot_exporter_impl : public server_snapshot_exporter {

    struct modified_components {
        std::array<unsigned short, sizeof...(Components)> time_remaining {};

        bool empty() const {
            return std::accumulate(time_remaining.begin(), time_remaining.end(), 0) == 0;
        }
    };

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        static const auto index = index_of_v<unsigned, Component, Components...>;

        if (auto *modified = registry.try_get<modified_components>(entity)) {
            modified->time_remaining[index] = 400;
        }
    }

public:
    server_snapshot_exporter_impl(entt::registry &registry,
                                  [[maybe_unused]] std::tuple<Components...>)
        : m_registry(&registry)
    {
        m_connections.push_back(registry.on_construct<networked_tag>().connect<&entt::registry::emplace<modified_components>>());
        ((m_connections.push_back(registry.on_update<Components>().template connect<&server_snapshot_exporter_impl<Components...>::template on_update<Components>>(*this))), ...);

        auto i = component_index_type{};
        (m_component_indices.emplace(entt::type_index<Components>(), i++), ...);
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
        auto &graph = registry.ctx().at<entity_graph>();
        auto node_view = registry.view<graph_node>();
        auto edge_view = registry.view<graph_edge>();
        auto owner_view = registry.view<entity_owner>();
        auto body_view = registry.view<position, orientation, linvel, angvel, dynamic_tag>();
        auto modified_view = registry.view<modified_components>();
        auto sleeping_view = registry.view<sleeping_tag>();
        bool allow_ownership = registry.get<remote_client>(dest_client_entity).allow_full_ownership;

        // Do not include input components of entities owned by destination
        // client as to not override client input on the client-side.
        // Clients own their input.
        // Only include entities which are in islands not fully owned by the client
        // since the server allows the client to have full control over entities in
        // the islands where there are no other clients present.
        for (auto entity : entities_of_interest) {
            if (!registry.valid(entity)) {
                continue;
            }

            if (sleeping_view.contains(entity)) {
                continue;
            }

            // Traverse entity graph using this entity as the starting point and
            // collect all owners (i.e. clients) that are reachable from this node.
            // If the only reachable client is the destination client, do not
            // include this entity in the packet because the client is allowed to
            // own the island when it's in it by itself.
            entity_graph::index_type node_index;
            bool dest_client_reachable = false;
            bool other_client_reachable = false;

            if (edge_view.contains(entity)) {
                auto [edge] = edge_view.get(entity);
                node_index = graph.edge_node_indices(edge.edge_index)[0];
            } else {
                auto [node] = node_view.get(entity);
                node_index = node.node_index;
            }

            bool temporary_ownership = false;

            if (allow_ownership) {
                graph.traverse(node_index, [&](auto node_index) {
                    auto neighbor = graph.node_entity(node_index);

                    if (owner_view.contains(neighbor)) {
                        auto [owner] = owner_view.get(neighbor);

                        if (owner.client_entity == dest_client_entity) {
                            dest_client_reachable = true;
                        } else if (owner.client_entity != entt::null) {
                            other_client_reachable = true;
                        }
                    }
                });

                // The client temporarily owns the entity if it's the only client
                // reachable through the graph.
                temporary_ownership = dest_client_reachable && !other_client_reachable;
            }

            auto owned_by_destination_client = owner_view.contains(entity) &&
                                               owner_view.get<entity_owner>(entity).client_entity == dest_client_entity;

            if (modified_view.contains(entity)) {
                auto [modified] = modified_view.get(entity);

                if (!modified.empty()) {
                    // Components that have been modified recently must be included in the
                    // packet, except if the client has temporary ownership of it or if it's
                    // an input component and the entity owner is the same as the destination
                    // client.
                    unsigned i = 0;
                    ((((modified.time_remaining[i] > 0 && !temporary_ownership &&
                        !(owned_by_destination_client && std::is_base_of_v<network_input, Components>) &&
                        !std::is_same_v<Components, action_history>) ?
                        internal::get_pool<Components>(snap.pools, i)->insert_single(registry, entity, snap.entities) : void(0)), ++i), ...);
                }
            }

            if (!temporary_ownership && body_view.contains(entity)) {
                internal::snapshot_insert_entity<position   >(*m_registry, entity, snap, index_of_v<unsigned, position,    Components...>);
                internal::snapshot_insert_entity<orientation>(*m_registry, entity, snap, index_of_v<unsigned, orientation, Components...>);
                internal::snapshot_insert_entity<linvel     >(*m_registry, entity, snap, index_of_v<unsigned, linvel,      Components...>);
                internal::snapshot_insert_entity<angvel     >(*m_registry, entity, snap, index_of_v<unsigned, angvel,      Components...>);
            }
        }
    }

    void export_comp_index(packet::registry_snapshot &snap, entt::entity entity,
                           const std::vector<component_index_type> &indices) const override {
        static const auto tuple = std::tuple<Components...>{};

        for (auto index : indices) {
            if (index < sizeof...(Components)) {
                visit_tuple(tuple, index, [&](auto &&c) {
                    using CompType = std::decay_t<decltype(c)>;
                    if (m_registry->all_of<CompType>(entity)) {
                        internal::snapshot_insert_entity<CompType>(*m_registry, entity, snap, index);
                    }
                });
            }
        }
    }

    void update(double time) override {
        EDYN_ASSERT(!(time < m_last_time));
        auto elapsed_ms = static_cast<unsigned>((time - m_last_time) * 1000u);
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
    double m_last_time {};
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
