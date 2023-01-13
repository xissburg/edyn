#ifndef EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP

#include <array>
#include <type_traits>
#include <entt/entity/fwd.hpp>
#include <utility>
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

/**
 * Keeps track of which components were modified during an extrapolation and
 * then inserts these into a registry operation so their new state can be
 * replicated in the main registry via the `extrapolation_result`.
 */
class extrapolation_modified_comp {
public:
    extrapolation_modified_comp(entt::registry &registry,
                                entt::sparse_set &relevant_entities,
                                entt::sparse_set &owned_entities)
        : m_registry(&registry)
        , m_relevant_entities(std::move(relevant_entities))
        , m_owned_entities(std::move(owned_entities))
    {}

    virtual ~extrapolation_modified_comp() = default;

    virtual void export_to_builder(registry_operation_builder &builder) = 0;

    // Mark all components in snapshot as modified to ensure they'll be included
    // in the call to `export_to_builder` even if they do not change during
    // extrapolation.
    virtual void mark_snapshot(entt::registry &registry, const packet::registry_snapshot &snapshot,
                               const entity_map &emap) = 0;

protected:
    entt::registry *m_registry;
    entt::sparse_set m_relevant_entities;
    entt::sparse_set m_owned_entities;
    std::vector<entt::scoped_connection> m_connections;
};

template<typename... Components>
class extrapolation_modified_comp_impl : public extrapolation_modified_comp {
    struct modified_components {
        std::array<uint16_t, sizeof...(Components)> indices;
        uint16_t count {};
    };

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        static const auto index = index_of_v<unsigned, Component, Components...>;

        if (auto *modified = registry.try_get<modified_components>(entity)) {
            bool found = false;

            for (unsigned i = 0; i < modified->count; ++i) {
                if (modified->indices[i] == index) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                modified->indices[modified->count++] = index;
            }
        }
    }

    template<typename Component>
    void observe_update(entt::registry &registry) {
        if constexpr(!std::is_empty_v<Component>) {
            m_connections.push_back(registry.on_update<Component>().template connect<&extrapolation_modified_comp_impl<Components...>::template on_update<Component>>(*this));
        }
    }

public:
    extrapolation_modified_comp_impl(entt::registry &registry,
                                     entt::sparse_set &relevant_entities,
                                     entt::sparse_set &owned_entities,
                                     [[maybe_unused]] std::tuple<Components...>)
        : extrapolation_modified_comp(registry, relevant_entities, owned_entities)
    {
        for (auto entity : m_relevant_entities) {
            registry.emplace<modified_components>(entity);
        }

        (observe_update<Components>(registry), ...);

        unsigned i = 0;
        ((m_is_network_input[i++] = std::disjunction_v<std::is_base_of<network_input, Components>, std::is_same<action_history, Components>>), ...);
    }

    void export_to_builder(registry_operation_builder &builder) override {
        const auto components_tuple = std::tuple<Components...>{};

        for (auto [entity, modified] : m_registry->view<modified_components>().each()) {
            const auto owned_entity = m_owned_entities.contains(entity);

            for (unsigned i = 0; i < modified.count; ++i) {
                auto comp_idx = modified.indices[i];

                // Do not include input components that belong to an owned entity.
                if (owned_entity && m_is_network_input[comp_idx]) {
                    continue;
                }

                visit_tuple(components_tuple, comp_idx, [&builder, entity = entity](auto &&c) {
                    using CompType = std::decay_t<decltype(c)>;
                    builder.replace<CompType>(entity);
                });
            }
        }
    }

    void mark_snapshot(entt::registry &registry, const packet::registry_snapshot &snapshot,
                       const entity_map &emap) override {
        for (auto &pool : snapshot.pools) {
            for (auto entity_index : pool.ptr->entity_indices) {
                auto remote_entity = snapshot.entities[entity_index];
                auto local_entity = emap.at(remote_entity);
                auto &modified = registry.get_or_emplace<modified_components>(local_entity);
                modified.indices[modified.count++] = pool.component_index;
            }
        }
    }

private:
    std::array<bool, sizeof...(Components)> m_is_network_input;
};

using make_extrapolation_modified_comp_func_t =
    std::unique_ptr<extrapolation_modified_comp>(entt::registry &, entt::sparse_set &, entt::sparse_set &);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
