#ifndef EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
#define EDYN_NETWORKING_COMP_STATE_HISTORY_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <vector>
#include <entt/core/type_info.hpp>
#include <entt/entity/fwd.hpp>
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/make_island_delta_builder.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn {

class comp_state_history {
    auto first_after(double timestamp) {
        return std::find_if(history.begin(), history.end(),
                            [timestamp] (auto &&elem) { return elem.timestamp > timestamp; });
    }

public:
    virtual ~comp_state_history() = default;

    using container_type = std::map<entt::id_type, std::unique_ptr<entity_component_container_base>>;

    struct element {
        container_type container;
        double timestamp;

        void import(entt::registry &registry, entity_map &emap, bool mark_dirty) const {
            for (auto &pair : container) {
                pair.second->import(registry, emap, mark_dirty);
            }
        }
    };

    virtual void take_snapshot(const entt::registry &registry,
                               const entt::sparse_set &entities,
                               container_type &container) const {};

    virtual void take_snapshot(const std::vector<pool_snapshot> &pools,
                               const entt::sparse_set &entities,
                               container_type &container) const {};

    template<typename DataSource>
    void emplace(const DataSource &source, const entt::sparse_set &entities, double timestamp) {
        // Insert input components of given entities from data source into container.
        auto container = container_type{};
        take_snapshot(source, entities, container);

        if (container.empty()) {
            return;
        }

        // Sorted insertion.
        std::lock_guard lock(mutex);
        auto it = first_after(timestamp);
        history.insert(it, {std::move(container), timestamp});
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
            if (elem.timestamp >= start_time + length_of_time) {
                break;
            }

            if (elem.timestamp >= start_time) {
                func(elem);
            }
        }
    }

    template<typename Func>
    void until(double time, Func func) const {
        std::lock_guard lock(mutex);

        for (auto &elem : history) {
            if (elem.timestamp >= time) {
                break;
            }

            func(elem);
        }
    }

protected:
    std::vector<element> history;
    mutable std::mutex mutex;
};

template<typename... Components>
class comp_state_history_impl : public comp_state_history {

    template<typename Component>
    void take_snapshot_single(const entt::registry &registry, const entt::sparse_set &entities, container_type &container) const {
        auto view = registry.view<Component>();

        for (auto entity : entities) {
            if (!view.contains(entity)) {
                continue;
            }

            auto [comp] = view.get(entity);
            auto id = entt::type_seq<Component>::value();
            auto &data = container[id];

            if (!data) {
                data.reset(new updated_entity_component_container<Component>{});
            }

            static_cast<updated_entity_component_container<Component> *>(data.get())->insert(entity, comp);
        }
    }

    template<typename Component>
    void take_snapshot_single(const pool_snapshot &pool, const entt::sparse_set &entities, container_type &container) const {
        if constexpr(!std::is_empty_v<Component>) {
            auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool.ptr.get());

            for (auto [entity, comp] : typed_pool->data) {
                if (!entities.contains(entity)) {
                    continue;
                }

                auto id = entt::type_seq<Component>::value();
                auto &data = container[id];

                if (!data) {
                    data.reset(new updated_entity_component_container<Component>{});
                }

                static_cast<updated_entity_component_container<Component> *>(data.get())->insert(entity, comp);
            }
        }
    }

public:
    void take_snapshot(const entt::registry &registry, const entt::sparse_set &entities,
                       container_type &container) const override {
        (take_snapshot_single<Components>(registry, entities, container), ...);
    }

    void take_snapshot(const std::vector<pool_snapshot> &pools, const entt::sparse_set &entities,
                       container_type &container) const override {
        for (auto &pool : pools) {
            ((entt::type_seq<Components>::value() == pool.ptr->get_type_id() ?
                take_snapshot_single<Components>(pool, entities, container) :
                void(0)), ...);
        }
    }
};

}

#endif // EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
