#ifndef EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP

#include <array>
#include <type_traits>
#include <entt/entity/fwd.hpp>
#include <utility>
#include "edyn/comp/action_list.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

/**
 * Keeps track of which components were modified during an extrapolation an
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

protected:
    entt::registry *m_registry;
    entt::sparse_set m_relevant_entities;
    entt::sparse_set m_owned_entities;
    std::vector<entt::scoped_connection> m_connections;
};

template<typename... Components>
class extrapolation_modified_comp_impl : public extrapolation_modified_comp {
    struct modified_components {
        std::array<bool, sizeof...(Components)> bits {};
    };

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        static constexpr auto index = index_of_v<unsigned, Component, Components...>;

        if (auto *modified = registry.try_get<modified_components>(entity)) {
            modified->bits[index] = true;
        }
    }

    template<typename Component, typename... Actions>
    constexpr bool is_action_list() {
        return std::disjunction_v<std::is_same<action_list<Actions>, Component>...>;
    }

public:
    template<typename... Actions>
    extrapolation_modified_comp_impl(entt::registry &registry,
                                     entt::sparse_set &relevant_entities,
                                     entt::sparse_set &owned_entities,
                                     [[maybe_unused]] std::tuple<Components...>,
                                     [[maybe_unused]] std::tuple<Actions...>)
        : extrapolation_modified_comp(registry, relevant_entities, owned_entities)
    {
        for (auto entity : m_relevant_entities) {
            registry.emplace<modified_components>(entity);
        }

        (m_connections.push_back(registry.on_update<Components>().template connect<&extrapolation_modified_comp_impl<Components...>::template on_update<Components>>(*this)), ...);

        unsigned i = 0;
        ((m_is_network_input[i++] = std::is_base_of_v<network_input, Components>), ...);

        i = 0;
        ((m_is_action_list[i++] = is_action_list<Components, Actions...>()), ...);
    }

    void export_to_builder(registry_operation_builder &builder) override {
        for (auto [entity, modified] : m_registry->view<modified_components>().each()) {
            unsigned i = 0;
            // Do not include input components that belong to an owned entity.
            (((modified.bits[i] && (!m_owned_entities.contains(entity) || !(m_is_network_input[i] || m_is_action_list[i])) ? builder.replace<Components>(entity) : void(0)), ++i), ...);
        }
    }

private:
    std::array<bool, sizeof...(Components)> m_is_network_input;
    std::array<bool, sizeof...(Components)> m_is_action_list;
};

using make_extrapolation_modified_comp_func_t =
    std::unique_ptr<extrapolation_modified_comp>(entt::registry &, entt::sparse_set &, entt::sparse_set &);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
