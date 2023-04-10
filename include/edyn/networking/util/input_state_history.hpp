#ifndef EDYN_NETWORKING_UTIL_INPUT_STATE_HISTORY_HPP
#define EDYN_NETWORKING_UTIL_INPUT_STATE_HISTORY_HPP

#include <entt/entity/fwd.hpp>
#include <memory>
#include <mutex>
#include <vector>
#include <type_traits>
#include <entt/core/type_info.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/action_list.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"

namespace edyn {

/**
 * @brief A history of user inputs and actions which will be applied during
 * extrapolation. It is accessed from multiple threads so it has to be thread-safe.
 */
class input_state_history {
public:
    // Record the state of input components and any new actions in the action
    // history of the provided entities.
    virtual void emplace(const entt::registry &registry,
                         const entt::sparse_set &entities, double timestamp) = 0;

    // Record state from a snapshot.
    virtual void emplace(const packet::registry_snapshot &snap,
                         const entt::sparse_set &entities,
                         double timestamp, double time_delta) = 0;

    // Erase all recorded input and actions until the given timestamp.
    virtual void erase_until(double timestamp) = 0;

    // Import all inputs and actions within the time range into the registry.
    // The entity map is necessary to map entities into the current registry
    // space, since all entities in the history are in the main registry space.
    virtual void import_each(double time, double length_of_time, entt::registry &registry, const entity_map &emap) const = 0;

    // Import all inputs that happened before `time` into the registry. This
    // ensures the correct initial value is assigned before the extrapolation
    // begins. Actions are not involved in this call. They're only imported
    // via `import_each`.
    virtual void import_latest(double time, entt::registry &registry, const entity_map &emap) const = 0;
};

template<typename... Inputs>
class input_state_history_impl : public input_state_history {

    template<typename Component>
    struct element {
        entt::entity entity;
        Component component;
        double timestamp;

        void import(entt::registry &registry, const entity_map &emap) const {
            auto remote_entity = entity;

            if (!emap.contains(remote_entity)) {
                return;
            }

            auto local_entity = emap.at(remote_entity);

            if (!registry.valid(local_entity)) {
                return;
            }

            auto comp = component;
            internal::map_child_entity(registry, emap, comp);

            if (registry.all_of<Component>(local_entity)) {
                registry.patch<Component>(local_entity, [&](auto &&current) {
                    merge_component(current, comp);
                });
            } else {
                registry.emplace<Component>(local_entity, comp);
            }
        }
    };

    template<typename Component>
    void add(const entt::registry &registry, const entt::sparse_set &entities, double timestamp) {
        auto view = registry.view<Component>();
        auto &history = std::get<std::vector<element<Component>>>(m_history);

        for (auto entity : entities) {
            if (view.contains(entity)) {
                auto [comp] = view.get(entity);
                history.push_back({entity, comp, timestamp});
            }
        }
    }

    template<typename Component>
    void add(const std::vector<entt::entity> &pool_entities, const pool_snapshot &pool_snapshot,
             const entt::sparse_set &entities, double timestamp) {
        auto &history = std::get<std::vector<element<Component>>>(m_history);
        auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool_snapshot.ptr.get());

        for (size_t i = 0; i < typed_pool->entity_indices.size(); ++i) {
            auto entity_index = typed_pool->entity_indices[i];
            auto entity = pool_entities[entity_index];
            auto &comp = typed_pool->components[i];

            if (entities.contains(entity)) {
                history.push_back({entity, comp, timestamp});
            }
        }
    }

    void add_actions(const entt::registry &registry, const entt::sparse_set &entities) {
        auto history_view = registry.view<action_history>();

        for (auto entity : entities) {
            if (history_view.contains(entity)) {
                auto [history] = history_view.get(entity);

                if (!history.empty()) {
                    if (!m_actions.contains(entity)) {
                        m_actions.emplace(entity, history);
                    } else {
                        m_actions.get(entity).merge(history);
                    }
                }
            }
        }
    }

    void add_actions(const std::vector<entt::entity> &pool_entities,
                     const pool_snapshot_data_impl<action_history> &pool_snapshot,
                     const entt::sparse_set &entities, double time_delta) {
        for (size_t i = 0; i < pool_snapshot.entity_indices.size(); ++i) {
            auto entity_index = pool_snapshot.entity_indices[i];
            auto entity = pool_entities[entity_index];
            auto comp = pool_snapshot.components[i];

            if (!comp.empty() && entities.contains(entity)) {
                for (auto &entry : comp.entries) {
                    entry.timestamp += time_delta;
                }

                if (!m_actions.contains(entity)) {
                    m_actions.emplace(entity, comp);
                } else {
                    m_actions.get(entity).merge(comp);
                }
            }
        }
    }

    template<typename Array>
    auto first_after(const Array &arr, double timestamp) const {
        return std::find_if(arr.begin(), arr.end(),
                            [timestamp](auto &&elem) { return elem.timestamp > timestamp; });
    }

    template<typename Component>
    void erase_until(double timestamp) {
        auto &history = std::get<std::vector<element<Component>>>(m_history);
        auto it = first_after(history, timestamp);
        history.erase(history.begin(), it);
    }

    template<typename Component>
    void import_each(double start_time, double length_of_time,
                     entt::registry &registry, const entity_map &emap) const {
        auto &history = std::get<std::vector<element<Component>>>(m_history);
        auto end_time = start_time + length_of_time;

        for (auto &elem : history) {
            if (elem.timestamp > end_time) {
                break;
            }

            if (elem.timestamp >= start_time) {
                elem.import(registry, emap);
            }
        }
    }

    template<typename Component>
    void import_latest(double time, entt::registry &registry, const entity_map &emap) const {
        auto &history = std::get<std::vector<element<Component>>>(m_history);

        for (auto &elem : history) {
            if (elem.timestamp > time) {
                break;
            }

            elem.import(registry, emap);
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

protected:
    void emplace(const entt::registry &registry,
                 const entt::sparse_set &entities, double timestamp) override {
        std::lock_guard lock(m_mutex);
        (add<Inputs>(registry, entities, timestamp), ...);
        add_actions(registry, entities);
    }

    void emplace(const packet::registry_snapshot &snap,
                 const entt::sparse_set &entities,
                 double timestamp, double time_delta) override {
        std::lock_guard lock(m_mutex);
        for (auto &pool : snap.pools) {
            ((entt::type_index<Inputs>::value() == pool.ptr->get_type_id() ?
                add<Inputs>(snap.entities, pool, entities, timestamp) :
                (void)0), ...);

            if (entt::type_index<action_history>::value() == pool.ptr->get_type_id()) {
                auto *typed_pool = static_cast<pool_snapshot_data_impl<action_history> *>(pool.ptr.get());
                add_actions(snap.entities, *typed_pool, entities, time_delta);
            }
        }
    }

    void erase_until(double timestamp) override {
        std::lock_guard lock(m_mutex);
        (erase_until<Inputs>(timestamp), ...);

        for (auto [entity, actions] : m_actions.each()) {
            actions.erase_until(timestamp);
        }
    }

    void import_each(double start_time, double length_of_time,
                     entt::registry &registry, const entity_map &emap) const override {
        std::lock_guard lock(m_mutex);
        (import_each<Inputs>(start_time, length_of_time, registry, emap), ...);

        if (m_import_action_func) {
            auto end_time = start_time + length_of_time;

            for (auto [entity, actions] : m_actions.each()) {
                for (auto &entry : actions.entries) {
                    if (entry.timestamp > end_time) {
                        break;
                    }

                    if (entry.timestamp >= start_time) {
                        auto remote_entity = entity;

                        if (emap.contains(remote_entity)) {
                            auto local_entity = emap.at(remote_entity);

                            if (registry.valid(local_entity)) {
                                (*m_import_action_func)(registry, local_entity, entry.action_index, entry.data);
                            }
                        }
                    }
                }
            }
        }
    }

    void import_latest(double time, entt::registry &registry, const entity_map &emap) const override {
        std::lock_guard lock(m_mutex);
        (import_latest<Inputs>(time, registry, emap), ...);
    }

public:
    template<typename... Actions>
    input_state_history_impl([[maybe_unused]] std::tuple<Inputs...>,
                             [[maybe_unused]] std::tuple<Actions...>) {
        if constexpr(sizeof...(Actions) > 0) {
            m_import_action_func = &import_action<Actions...>;
        }
    }

private:
    std::tuple<std::vector<element<Inputs>>...> m_history;
    entt::storage<action_history> m_actions;
    mutable std::mutex m_mutex;

    using import_action_func_t = void(entt::registry &, entt::entity,
                                      action_history::action_index_type,
                                      const std::vector<uint8_t> &);
    import_action_func_t *m_import_action_func {nullptr};
};

}

#endif // EDYN_NETWORKING_UTIL_INPUT_STATE_HISTORY_HPP
