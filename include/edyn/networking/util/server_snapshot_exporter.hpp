#ifndef EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/registry.hpp>
#include <entt/signal/sigh.hpp>
#include <numeric>
#include <type_traits>
#include <vector>
#include <utility>
#include "edyn/comp/action_list.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/child_list.hpp"
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
#include "edyn/networking/comp/exporter_modified_components.hpp"
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

    using modified_components = exporter_modified_components<sizeof...(Components)>;

    template<typename... Cs>
    void bump_component(modified_components &modified) const {
        (modified.bump_index(index_of_v<unsigned, Cs, Components...>), ...);
    }

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        auto modified_view = registry.view<modified_components>();
        if (modified_view.contains(entity)) {
            auto [modified] = modified_view.get(entity);
            bump_component<Component>(modified);
        }
    }

    template<typename Component>
    void observe_update(entt::registry &registry) {
        if constexpr(!std::is_empty_v<Component>) {
            m_connections.push_back(registry.on_update<Component>().template connect<&server_snapshot_exporter_impl<Components...>::template on_update<Component>>(*this));
        }
    }

    void export_modified_entity(const entt::registry &registry, entt::entity entity,
                                const modified_components &modified,
                                bool owned_by_destination,
                                packet::registry_snapshot &snap) const {
        static const auto components_tuple = std::tuple<Components...>{};

        for (unsigned i = 0; i < modified.count; ++i) {
            auto comp_index = modified.entry[i].index;
            visit_tuple(components_tuple, comp_index, [&](auto &&c) {
                using CompType = std::decay_t<decltype(c)>;
                constexpr auto is_input = std::is_base_of_v<network_input, CompType>;
                constexpr auto is_action = std::is_same_v<CompType, action_history>;

                // Must not send action history or input state of entities
                // owned by destination client.
                if (!(is_action && owned_by_destination) && !(is_input && owned_by_destination)) {
                    internal::snapshot_insert_entity<CompType>(registry, entity, snap, comp_index);
                }
            });
        }
    }

    template<typename Component, typename It>
    void export_single(packet::registry_snapshot &snap, It first, It last, size_t index) const {
        constexpr auto is_action = std::is_same_v<Component, action_history>;

        // Actions must not be shared among clients.
        if constexpr(!is_action) {
            for (; first != last; ++first) {
                auto entity = *first;

                if (m_registry->all_of<Component>(entity)) {
                    internal::snapshot_insert_entity<Component>(*m_registry, entity, snap, index);
                }
            }
        }
    }

    template<typename It, size_t... Indexes>
    void export_all_indices(packet::registry_snapshot &snap, It first, It last, std::index_sequence<Indexes...>) const {
        (export_single<Components>(snap, first, last, Indexes), ...);
    }

public:
    server_snapshot_exporter_impl(entt::registry &registry,
                                  [[maybe_unused]] std::tuple<Components...>)
        : m_registry(&registry)
    {
        m_connections.push_back(registry.on_construct<networked_tag>().connect<&entt::registry::emplace<modified_components>>());
        (observe_update<Components>(registry), ...);

        auto i = component_index_type{};
        (m_component_indices.emplace(entt::type_index<Components>(), i++), ...);
    }

    template<typename It>
    void export_all(packet::registry_snapshot &snap, It first, It last) const {
        constexpr auto indices = std::make_index_sequence<sizeof...(Components)>();
        export_all_indices(snap, first, last, indices);
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
        auto &graph = registry.ctx().get<entity_graph>();
        auto node_view = registry.view<graph_node>();
        auto edge_view = registry.view<graph_edge>();
        auto owner_view = registry.view<const entity_owner>();
        auto modified_view = registry.view<const modified_components>();
        auto parent_view = registry.view<parent_comp>();
        auto child_view = registry.view<child_list>();

        bool allow_ownership = registry.get<remote_client>(dest_client_entity).allow_full_ownership;

        // Do not include input components of entities owned by destination
        // client as to not override client input on the client-side.
        // Clients own their input.
        // Only include entities which are in islands not fully owned by the client
        // since the server allows the client to have full control over entities in
        // the islands where there are no other clients present.
        for (auto entity : entities_of_interest) {
            if (!registry.valid(entity) || !modified_view.contains(entity)) {
                continue;
            }

            auto [modified] = modified_view.get(entity);
            const auto is_parent = parent_view.contains(entity);

            // Parent could have a modified child.
            if (modified.count == 0 && !is_parent) {
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

            if (temporary_ownership) {
                // Entity is temporarily owned by the destination client, which
                // means the server applies state set by client directly and so
                // does not send state back to this client which would override
                // the state they're controlling.
                continue;
            }

            const auto owned_by_destination =
                owner_view.contains(entity) &&
                std::get<0>(owner_view.get(entity)).client_entity == dest_client_entity;
            export_modified_entity(registry, entity, modified, owned_by_destination, snap);

            // Include child entities
            if (is_parent) {
                auto [parent] = parent_view.get(entity);
                auto child_entity = parent.child;

                while (child_entity != entt::null) {
                    if (modified_view.contains(child_entity)) {
                        auto [child_modified] = modified_view.get(child_entity);
                        export_modified_entity(registry, child_entity, child_modified, owned_by_destination, snap);
                    }

                    auto [child] = child_view.get(child_entity);
                    child_entity = child.next;
                }
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

        auto modified_view = m_registry->view<modified_components>();
        auto dynamic_view = m_registry->view<dynamic_tag>(exclude_sleeping_disabled);

        for (auto [entity, modified] : modified_view.each()) {
            modified.decay(elapsed_ms);

            // If it's a dynamic rigid body that's awake, mark transforms and velocities
            // as modified because these components change in every step of the simulation
            // and have their value assigned directly to avoid triggering the `on_update`
            // signals every time.
            if (dynamic_view.contains(entity)) {
                bump_component<position, orientation, linvel, angvel>(modified);
            }
        }
    }

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    double m_last_time {};
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_SNAPSHOT_EXPORTER_HPP
