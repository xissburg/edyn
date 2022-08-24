#ifndef EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP

#include <array>
#include <type_traits>
#include <entt/entity/fwd.hpp>
#include <utility>
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
    extrapolation_modified_comp(entt::registry &registry)
        : m_registry(&registry)
    {}

    virtual ~extrapolation_modified_comp() = default;

    virtual void export_to_builder(registry_operation_builder &builder) = 0;

protected:
    entt::registry *m_registry;
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

public:
    extrapolation_modified_comp_impl(entt::registry &registry,
                                     const std::vector<entt::entity> &relevant_entities,
                                     [[maybe_unused]] std::tuple<Components...>)
        : extrapolation_modified_comp(registry)
    {
        for (auto entity : relevant_entities) {
            registry.emplace<modified_components>(entity);
        }

        (m_connections.push_back(registry.on_update<Components>().template connect<&extrapolation_modified_comp_impl<Components...>::template on_update<Components>>(*this)), ...);
    }

    void export_to_builder(registry_operation_builder &builder) override {
        for (auto [entity, modified] : m_registry->view<modified_components>().each()) {
            unsigned i = 0;
            (((modified.bits[i] ? builder.replace<Components>(entity) : void(0)), ++i), ...);
        }
    }
};

using make_extrapolation_modified_comp_func_t = std::unique_ptr<extrapolation_modified_comp>(entt::registry &, const std::vector<entt::entity> &);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
