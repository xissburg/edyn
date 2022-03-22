#ifndef EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
#define EDYN_NETWORKING_COMP_STATE_HISTORY_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <vector>
#include <entt/core/type_info.hpp>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/util/registry_snapshot.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

class comp_state_history {
    auto first_after(double timestamp) {
        return std::find_if(history.begin(), history.end(),
                            [timestamp] (auto &&elem) { return elem.timestamp > timestamp; });
    }

protected:
    virtual registry_operation_collection
    take_snapshot(const entt::registry &registry, const entt::sparse_set &entities) {
        return {};
    }

    virtual registry_operation_collection
    take_snapshot(const registry_snapshot &snap, const entt::sparse_set &entities) {
        return {};
    }

public:
    virtual ~comp_state_history() = default;

    struct element {
        registry_operation_collection ops;
        double timestamp;
    };

    template<typename DataSource>
    void emplace(const DataSource &source, const entt::sparse_set &entities, double timestamp) {
        // Insert input components of given entities from data source into container.
        auto ops = take_snapshot(source, entities);

        if (ops.empty()) {
            return;
        }

        // Sorted insertion.
        std::lock_guard lock(mutex);
        auto it = first_after(timestamp);
        history.insert(it, {std::move(ops), timestamp});
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
    void take_snapshot_single(const std::vector<entt::entity> &pool_entities,
                              const pool_snapshot &pool,
                              const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Component>) {
            auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool.ptr.get());

            for (size_t i = 0; i < typed_pool->entity_indices.size(); ++i) {
                auto entity_index = typed_pool->entity_indices[i];
                auto entity = pool_entities[entity_index];
                auto &comp = typed_pool->components[i];

                if (entities.contains(entity)) {
                    op_builder.replace(entity, comp);
                }
            }
        }
    }

protected:
    registry_operation_collection
    take_snapshot(const entt::registry &registry, const entt::sparse_set &entities) override {
        op_builder.replace_all(registry, entities);
        return op_builder.finish();
    }

    registry_operation_collection
    take_snapshot(const registry_snapshot &snap, const entt::sparse_set &entities) override {
        for (auto &pool : snap.pools) {
            ((entt::type_seq<Components>::value() == pool.ptr->get_type_id() ?
                take_snapshot_single<Components>(snap.entities, pool, entities) :
                void(0)), ...);
        }
        return op_builder.finish();
    }

private:
    registry_operation_builder_impl<Components...> op_builder;
};

}

#endif // EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
