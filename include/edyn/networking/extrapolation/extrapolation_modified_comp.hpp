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
    extrapolation_modified_comp(entt::registry &registry)
        : m_registry(&registry)
    {}

    virtual ~extrapolation_modified_comp() = default;

    virtual void set_observe_changes(bool observe) = 0;
    virtual void add_entity(entt::entity entity, bool owned_by_client) = 0;
    virtual void remove_entity(entt::entity entity) = 0;
    virtual void clear_modified(const entt::sparse_set &entities) = 0;
    virtual void export_to_builder(registry_operation_builder &builder, const entt::sparse_set &entities) = 0;
    virtual void import_remote_state(const entt::sparse_set &entities) = 0;
    virtual void export_remote_state(const entt::sparse_set &entities) = 0;

protected:
    entt::registry *m_registry;
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
                                     [[maybe_unused]] std::tuple<Components...>)
        : extrapolation_modified_comp(registry)
    {
        unsigned i = 0;
        ((m_is_network_input[i++] = std::disjunction_v<std::is_base_of<network_input, Components>, std::is_same<action_history, Components>>), ...);
    }

    void set_observe_changes(bool observe) override {
        if (observe) {
            (observe_update<Components>(*m_registry), ...);
        } else {
            m_connections.clear();
        }
    }

    void add_entity(entt::entity entity, bool owned_by_client) override {
        m_registry->emplace<modified_components>(entity);

        if (owned_by_client) {
            m_owned_entities.emplace(entity);
        }
    }

    void remove_entity(entt::entity entity) override {
        m_registry->erase<modified_components>();
        m_owned_entities.remove(entity);
    }

    void clear_modified(const entt::sparse_set &entities) override {
        auto modified_view = m_registry->view<modified_components>();

        for (auto entity : entities) {
            auto [modified] = modified_view.get(entity);
            modified.count = 0;
        }
    }

    void export_to_builder(registry_operation_builder &builder, const entt::sparse_set &entities) override {
        const auto components_tuple = std::tuple<Components...>{};
        auto modified_view = m_registry->view<modified_components>();

        for (auto entity : entities) {
            auto [modified] = modified_view.get(entity);
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

    template<typename Component>
    void import_remote_state_single(const entt::sparse_set &entities) {
        using namespace entt::literals;
        auto storage = m_registry->storage<Component>(hash_combine("server"_hs, entt::type_hash<Component>()));
        auto view = m_registry->view<Component>();

        for (auto entity : entities) {
            if (view.contains(entity)) {
                auto [comp] = view.get(entity);
                comp = storage.get(entity);
            }
        }
    }

    // Copy values from the "server" named storage into the main default storage,
    // effectively overriding all components with the last seen values from server.
    void import_remote_state(const entt::sparse_set &entities) override {
        (import_remote_state_single<Components>(entities), ...);
    }

    // Copy values from main storage into "server" named storage.
    void export_remote_state(const entt::sparse_set &entities) override {

    }

private:
    std::array<bool, sizeof...(Components)> m_is_network_input;
};

using make_extrapolation_modified_comp_func_t =
    std::unique_ptr<extrapolation_modified_comp>(entt::registry &, entt::sparse_set &, entt::sparse_set &);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
