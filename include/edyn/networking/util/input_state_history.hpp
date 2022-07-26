#ifndef EDYN_NETWORKING_INPUT_STATE_HISTORY_HPP
#define EDYN_NETWORKING_INPUT_STATE_HISTORY_HPP

#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>
#include <entt/core/type_info.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/action_list.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"

namespace edyn {

namespace detail {
    class comp_state_pool {
    public:
        virtual ~comp_state_pool() = default;
        virtual void import(entt::registry &, const entity_map &) const = 0;
        virtual bool empty() const = 0;
        virtual entt::id_type type_id() const = 0;
    };

    template<typename Component>
    class comp_state_pool_impl : public comp_state_pool {
    public:
        void import(entt::registry &registry, const entity_map &emap) const override {
            for (auto &pair : m_data) {
                auto remote_entity = pair.first;

                if (!emap.contains(remote_entity)) {
                    continue;
                }

                auto local_entity = emap.at(remote_entity);

                if (!registry.valid(local_entity)) {
                    continue;
                }

                auto comp = pair.second;
                internal::map_child_entity(registry, emap, comp);

                if (registry.all_of<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, comp);
                    registry.get_or_emplace<dirty>(local_entity).template updated<Component>();
                } else {
                    registry.emplace<Component>(local_entity, comp);
                    registry.get_or_emplace<dirty>(local_entity).template created<Component>();
                }
            }
        }

        bool empty() const override {
            return m_data.empty();
        }

        void insert(entt::entity entity, const Component &comp) {
            m_data.emplace_back(entity, comp);
        }

        entt::id_type type_id() const override {
            return entt::type_index<Component>::value();
        }

    private:
        std::vector<std::pair<entt::entity, Component>> m_data;
    };
}

/**
 * @brief A history of user inputs which will be applied during extrapolation.
 * It is accessed from multiple threads so it has to be thread-safe.
 */
class input_state_history {
    auto first_after(double timestamp) {
        return std::find_if(history.begin(), history.end(),
                            [timestamp](auto &&elem) { return elem.timestamp > timestamp; });
    }

public:
    struct snapshot {
        std::vector<std::unique_ptr<detail::comp_state_pool>> pools;
    };

    struct element {
        input_state_history::snapshot snapshot;
        double timestamp;

        void import(entt::registry &registry, const entity_map &emap) const {
            for (auto &pool : snapshot.pools) {
                pool->import(registry, emap);
            }
        }
    };

    /**
     * Even though the timestamp of a registry snapshot lies right after the time
     * an action happenend, it is possible that the action wasn't still applied
     * in the server side at the time the snapshot was generated. Perhaps the
     * action was applied at the same time the snapshot was generated and then
     * its effects will only become visible in the next update, which will cause
     * a glitch on client-side extrapolation because the action will not be
     * applied initially and the initial state does not include the effects of
     * the action because it wasn't applied in the server at the time the
     * snapshot was generated. All actions that happened before the snapshot
     * time within this threshold will be applied at the start of an extrapolation.
     */
    double action_time_threshold{0.06};

protected:
    virtual snapshot take_snapshot(const entt::registry &registry,
                                   const entt::sparse_set &entities) const {
        return {};
    }

    virtual snapshot take_snapshot(const packet::registry_snapshot &snap,
                                   const entt::sparse_set &entities) const {
        return {};
    }

public:
    virtual ~input_state_history() = default;

    template<typename DataSource>
    void emplace(const DataSource &source, const entt::sparse_set &entities, double timestamp) {
        // Insert input components of given entities from data source into container.
        auto snapshot = take_snapshot(source, entities);

        if (snapshot.pools.empty()) {
            return;
        }

        // Sorted insertion.
        std::lock_guard lock(mutex);
        auto it = first_after(timestamp);
        history.insert(it, {std::move(snapshot), timestamp});
    }

    void erase_until(double timestamp) {
        std::lock_guard lock(mutex);
        auto it = first_after(timestamp);
        history.erase(history.begin(), it);
    }

    template<typename Func>
    void each(double start_time, double length_of_time, Func func) const {
        std::lock_guard lock(mutex);

        for (auto &elem : history) {
            if (elem.timestamp > start_time + length_of_time) {
                break;
            }

            if (elem.timestamp >= start_time) {
                func(elem);
            }
        }
    }

    void import_each(double time, double length_of_time, entt::registry &registry, const entity_map &emap) const {
        each(time, length_of_time, [&](auto &&elem) {
            elem.import(registry, emap);
        });
    }

    virtual void import_initial_state(entt::registry &registry, const entity_map &emap, double time) {}

protected:
    std::vector<element> history;
    mutable std::mutex mutex;
};

template<typename... Inputs>
class input_state_history_impl : public input_state_history {

    template<typename Input>
    static void add_to_snapshot(snapshot &snapshot, const entt::registry &registry,
                                const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Input>) {
            auto view = registry.view<Input>();
            auto pool = std::unique_ptr<detail::comp_state_pool_impl<Input>>{};

            for (auto entity : entities) {
                if (view.contains(entity)) {
                    if (!pool) {
                        pool.reset(new detail::comp_state_pool_impl<Input>);
                    }

                    auto [comp] = view.get(entity);
                    pool->insert(entity, comp);
                }
            }

            if (pool && !pool->empty()) {
                snapshot.pools.push_back(std::move(pool));
            }
        }
    }

    template<typename Input>
    static void add_to_snapshot(snapshot &snapshot, const std::vector<entt::entity> &pool_entities,
                                const pool_snapshot &pool_snapshot, const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Input>) {
            auto pool = std::unique_ptr<detail::comp_state_pool_impl<Input>>{};
            auto *typed_pool = static_cast<pool_snapshot_data_impl<Input> *>(pool_snapshot.ptr.get());

            for (size_t i = 0; i < typed_pool->entity_indices.size(); ++i) {
                auto entity_index = typed_pool->entity_indices[i];
                auto entity = pool_entities[entity_index];
                auto &comp = typed_pool->components[i];

                if (entities.contains(entity)) {
                    if (!pool) {
                        pool.reset(new detail::comp_state_pool_impl<Input>);
                    }

                    pool->insert(entity, comp);
                }
            }

            if (pool && !pool->empty()) {
                snapshot.pools.push_back(std::move(pool));
            }
        }
    }

protected:
    snapshot take_snapshot(const entt::registry &registry,
                           const entt::sparse_set &entities) const override {
        snapshot snapshot;
        (add_to_snapshot<Inputs>(snapshot, registry, entities), ...);
        return snapshot;
    }

    snapshot take_snapshot(const packet::registry_snapshot &snap,
                           const entt::sparse_set &entities) const override {
        snapshot snapshot;
        for (auto &pool : snap.pools) {
            ((entt::type_index<Inputs>::value() == pool.ptr->get_type_id() ?
                add_to_snapshot<Inputs>(snapshot, snap.entities, pool, entities) :
                (void)0), ...);
        }
        return snapshot;
    }

    // Import the last state of all inputs right before the extrapolation start time.
    // This ensures correct initial input state before extrapolation begins.
    template<typename Input>
    struct import_initial_state_single {
        static void import(entt::registry &registry, const entity_map &emap,
                           const std::vector<element> &history, double time,
                           double action_threshold) {
            // Find history element with the greatest timestamp smaller than `time`
            // which contains a pool of the given component type.
            for (auto i = history.size(); i > 0; --i) {
                auto &elem = history[i-1];

                if (elem.timestamp > time) {
                    continue;
                }

                auto pool_it = std::find_if(
                    elem.snapshot.pools.begin(), elem.snapshot.pools.end(),
                    [](auto &&pool) {
                        return pool->type_id() == entt::type_index<Input>::value();
                    });

                if (pool_it != elem.snapshot.pools.end()) {
                    (*pool_it)->import(registry, emap);
                    break;
                }
            }
        }
    };

    // Apply all actions that happened slightly before the extrapolation start time.
    // This prevents glitches due to the effect of actions not yet being present
    // in a registry snapshot due to small timing errors.
    template<typename Action>
    struct import_initial_state_single<action_list<Action>> {
        static void import(entt::registry &registry, const entity_map &emap,
                           const std::vector<element> &history, double time,
                           double action_threshold) {
            for (auto &elem : history) {
                if (elem.timestamp < time && time - elem.timestamp < action_threshold) {
                    auto pool_it = std::find_if(
                        elem.snapshot.pools.begin(), elem.snapshot.pools.end(),
                        [](auto &&pool) {
                            return pool->type_id() == entt::type_index<action_list<Action>>::value();
                        });

                    if (pool_it != elem.snapshot.pools.end()) {
                        (*pool_it)->import(registry, emap);
                    }
                }
            }
        }
    };

public:
    input_state_history_impl() = default;
    input_state_history_impl([[maybe_unused]] std::tuple<Inputs...>) {}

    void import_initial_state(entt::registry &registry, const entity_map &emap, double time) override {
        std::lock_guard lock(mutex);
        (import_initial_state_single<Inputs>::import(registry, emap, history, time, action_time_threshold), ...);
    }
};

}

#endif // EDYN_NETWORKING_INPUT_STATE_HISTORY_HPP
