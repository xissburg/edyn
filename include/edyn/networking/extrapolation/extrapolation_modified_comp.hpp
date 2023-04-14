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

    // Start observing changes to relevant components to keep track of which
    // components changed during extrapolation.
    virtual void set_observe_changes(bool observe) = 0;

    // Add an entity to be observed for changes.
    virtual void add_entity(entt::entity entity) = 0;

    // Remove entity, usually before being destroyed.
    virtual void remove_entity(entt::entity entity) = 0;

    // Load initial state prior to extrapolation into the target registry.
    virtual void import_remote_state(const entt::sparse_set &entities) = 0;

    // Store most recently seen remote state into the local storage.
    virtual void export_remote_state(const entt::sparse_set &entities) = 0;

    // Exports components that have been modified throughout extrapolation into
    // the builder.
    virtual void export_to_builder(registry_operation_builder &builder,
                                   const entt::sparse_set &owned_entities) = 0;

    // Clears the modified flags. Should be called after exporting to a builder
    // so the state is cleared for future extrapolations.
    virtual void clear_modified() = 0;

protected:
    entt::registry *m_registry;
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

    template<typename Component>
    auto & assure() {
        entt::id_type id = entt::type_hash<Component>::value();
        auto &&pool = remote_state_pools[id];

        if(!pool) {
            pool.reset(new entt::storage<Component>{});
        }

        return static_cast<entt::storage<Component> &>(*pool);
    }

    template<typename Component>
    void import_remote_state_single(const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Component>) {
            auto &storage = assure<Component>();
            auto view = m_registry->view<Component>();

            for (auto entity : entities) {
                if (view.contains(entity) && storage.contains(entity)) {
                    auto [comp] = view.get(entity);
                    comp = storage.get(entity);
                }
            }
        }
    }

    template<typename Component>
    void export_remote_state_single(const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Component>) {
            auto &storage = assure<Component>();
            auto view = m_registry->view<Component>();

            for (auto entity : entities) {
                if (view.contains(entity)) {
                    auto &comp = view.template get<Component>(entity);

                    if (storage.contains(entity)) {
                        storage.get(entity) = comp;
                    } else {
                        storage.emplace(entity, comp);
                    }
                }
            }
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

    void add_entity(entt::entity entity) override {
        m_registry->emplace<modified_components>(entity);
    }

    void remove_entity(entt::entity entity) override {
        if (m_registry->valid(entity)) {
            m_registry->erase<modified_components>(entity);
        }
        (assure<Components>().remove(entity), ...);
    }

    // Copy values from local storage into registry, effectively overriding
    // all components with the last seen values from server. Must be called
    // prior to extrapolating to set the initial state.
    void import_remote_state(const entt::sparse_set &entities) override {
        (import_remote_state_single<Components>(entities), ...);
    }

    // Copy values from registry into local storage. Must be called after
    // importing new remote state into the extrapolator's registry.
    void export_remote_state(const entt::sparse_set &entities) override {
        (export_remote_state_single<Components>(entities), ...);
    }

    void export_to_builder(registry_operation_builder &builder,
                           const entt::sparse_set &owned_entities) override {
        const auto components_tuple = std::tuple<Components...>{};

        for (auto [entity, modified] : m_registry->view<modified_components>().each()) {
            const auto owned_entity = owned_entities.contains(entity);

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

    void clear_modified() override {
        m_registry->view<modified_components>().each([](auto &&modified) {
            modified.count = 0;
        });
    }

private:
    std::array<bool, sizeof...(Components)> m_is_network_input;
    entt::dense_map<entt::id_type, std::unique_ptr<entt::sparse_set>, entt::identity> remote_state_pools;
};

using make_extrapolation_modified_comp_func_t =
    std::unique_ptr<extrapolation_modified_comp>(entt::registry &);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_MODIFIED_COMP_HPP
