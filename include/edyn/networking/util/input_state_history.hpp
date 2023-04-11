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

namespace internal {
    template<typename Component>
    struct component_history {
        struct entry {
            Component component;
            double timestamp;
        };

        std::vector<entry> entries;
    };
}

/**
 * @brief A history of user inputs and actions which will be applied during
 * extrapolation. It is accessed from multiple threads so it has to be thread-safe.
 */
template<typename... Inputs>
struct input_state_history {
    entt::storage<action_history> actions;
    std::tuple<entt::storage<internal::component_history<Inputs>>...> inputs;
    std::mutex mutex;

    input_state_history() = default;
    input_state_history(input_state_history &) = default;
    input_state_history(input_state_history &&) = default;
    input_state_history & operator=(input_state_history &) = default;
    input_state_history & operator=(input_state_history &&) = default;
    input_state_history([[maybe_unused]] const std::tuple<Inputs...> &) {}

    template<typename Input>
    auto & get_inputs() {
        return std::get<entt::storage<internal::component_history<Input>>>(inputs);
    }
};

class input_state_history_writer {
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

    // Removes an entity from internal storages in case it had been added before.
    // Must be called for all entities that have been destroyed.
    virtual void remove_entity(entt::entity entity) = 0;
};

class input_state_history_reader {
public:
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
class input_state_history_writer_impl : public input_state_history_writer {

    template<typename Component>
    void add(const entt::registry &registry, const entt::sparse_set &entities, double timestamp) {
        auto view = registry.view<Component>();
        auto &inputs = m_history->template get_inputs<Component>();

        for (auto entity : entities) {
            if (view.contains(entity)) {
                if (!inputs.contains(entity)) {
                    inputs.emplace(entity);
                }

                auto [comp] = view.get(entity);
                inputs.get(entity).entries.push_back({comp, timestamp});
            }
        }
    }

    template<typename Component>
    void add(const std::vector<entt::entity> &pool_entities, const pool_snapshot &pool_snapshot,
             const entt::sparse_set &entities, double timestamp) {
        auto &inputs = m_history->template get_inputs<Component>();
        auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool_snapshot.ptr.get());

        for (size_t i = 0; i < typed_pool->entity_indices.size(); ++i) {
            auto entity_index = typed_pool->entity_indices[i];
            auto entity = pool_entities[entity_index];
            auto &comp = typed_pool->components[i];

            if (entities.contains(entity)) {
                if (!inputs.contains(entity)) {
                    inputs.emplace(entity);
                }

                inputs.get(entity).entries.push_back({comp, timestamp});
            }
        }
    }

    void add_actions(const entt::registry &registry, const entt::sparse_set &entities) {
        auto history_view = registry.view<action_history>();

        for (auto entity : entities) {
            if (history_view.contains(entity)) {
                auto [history] = history_view.get(entity);

                if (!history.empty()) {
                    if (!m_history->actions.contains(entity)) {
                        m_history->actions.emplace(entity, history);
                    } else {
                        m_history->actions.get(entity).merge(history);
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

                if (!m_history->actions.contains(entity)) {
                    m_history->actions.emplace(entity, comp);
                } else {
                    m_history->actions.get(entity).merge(comp);
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
        auto &inputs = m_history->template get_inputs<Component>();

        for (auto [entity, history] : inputs.each()) {
            auto it = first_after(history.entries, timestamp);
            history.entries.erase(history.entries.begin(), it);
        }
    }

public:
    input_state_history_writer_impl(std::shared_ptr<input_state_history<Inputs...>> history)
        : m_history(history)
    {}

    void emplace(const entt::registry &registry,
                 const entt::sparse_set &entities, double timestamp) override {
        std::lock_guard lock(m_history->mutex);
        (add<Inputs>(registry, entities, timestamp), ...);
        add_actions(registry, entities);
    }

    void emplace(const packet::registry_snapshot &snap,
                 const entt::sparse_set &entities,
                 double timestamp, double time_delta) override {
        std::lock_guard lock(m_history->mutex);
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
        std::lock_guard lock(m_history->mutex);
        (erase_until<Inputs>(timestamp), ...);

        for (auto [entity, actions] : m_history->actions.each()) {
            actions.erase_until(timestamp);
        }
    }

    void remove_entity(entt::entity entity) override {
        std::lock_guard lock(m_history->mutex);

        m_history->actions.remove(entity);
        (m_history->template get_inputs<Inputs>().remove(entity), ...);
    }

private:
    std::shared_ptr<input_state_history<Inputs...>> m_history;
};

template<typename... Inputs>
class input_state_history_reader_impl : public input_state_history_reader {

    template<typename Component>
    void import_component(entt::registry &registry, entt::entity remote_entity,
                          Component comp, const entity_map &emap) const {
        if (!emap.contains(remote_entity)) {
            return;
        }

        auto local_entity = emap.at(remote_entity);

        if (!registry.valid(local_entity)) {
            return;
        }

        internal::map_child_entity(registry, emap, comp);

        if (registry.all_of<Component>(local_entity)) {
            registry.patch<Component>(local_entity, [&](auto &&current) {
                merge_component(current, comp);
            });
        } else {
            registry.emplace<Component>(local_entity, comp);
        }
    }

    template<typename Component>
    void import_each_input(double start_time, double length_of_time,
                           entt::registry &registry, const entity_map &emap) const {
        auto &inputs = m_history->template get_inputs<Component>();
        auto end_time = start_time + length_of_time;

        for (auto [entity, history] : inputs.each()) {
            for (auto &entry : history.entries) {
                if (entry.timestamp > end_time) {
                    break;
                }

                if (entry.timestamp >= start_time) {
                    import_component(registry, entity, entry.component, emap);
                }
            }
        }
    }

    void import_each_action(double start_time, double length_of_time,
                            entt::registry &registry, const entity_map &emap) const {
        if (!m_import_action_func) {
            return;
        }

        auto end_time = start_time + length_of_time;

        for (auto [entity, actions] : m_history->actions.each()) {
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

    template<typename Array>
    auto first_before(const Array &arr, double timestamp) const {
        return std::find_if(arr.rbegin(), arr.rend(),
                            [timestamp](auto &&elem) { return elem.timestamp <= timestamp; });
    }

    template<typename Component>
    void import_latest_inputs(double time, entt::registry &registry, const entity_map &emap) const {
        auto &inputs = m_history->template get_inputs<Component>();

        for (auto [entity, history] : inputs.each()) {
            // Import the first component state that's before the given time.
            auto it = first_before(history.entries, time);

            if (it != history.entries.rend()) {
                import_component(registry, entity, it->component, emap);
            }
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

public:
    template<typename... Actions>
    input_state_history_reader_impl(std::shared_ptr<input_state_history<Inputs...>> history,
                                    [[maybe_unused]] std::tuple<Actions...>)
        : m_history(history)
    {
        if constexpr(sizeof...(Actions) > 0) {
            m_import_action_func = &import_action<Actions...>;
        }
    }

    void import_each(double start_time, double length_of_time,
                     entt::registry &registry, const entity_map &emap) const override {
        std::lock_guard lock(m_history->mutex);
        (import_each_input<Inputs>(start_time, length_of_time, registry, emap), ...);
        import_each_action(start_time, length_of_time, registry, emap);
    }

    void import_latest(double time, entt::registry &registry, const entity_map &emap) const override {
        std::lock_guard lock(m_history->mutex);
        (import_latest_inputs<Inputs>(time, registry, emap), ...);
    }

private:
    std::shared_ptr<input_state_history<Inputs...>> m_history;

    using import_action_func_t = void(entt::registry &, entt::entity,
                                      action_history::action_index_type,
                                      const std::vector<uint8_t> &);
    import_action_func_t *m_import_action_func {nullptr};
};

}

#endif // EDYN_NETWORKING_UTIL_INPUT_STATE_HISTORY_HPP
