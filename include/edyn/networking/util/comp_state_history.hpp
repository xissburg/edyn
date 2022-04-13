#ifndef EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
#define EDYN_NETWORKING_COMP_STATE_HISTORY_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <vector>
#include <entt/core/type_info.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"

namespace edyn {

namespace detail {
    class comp_state_pool {
    public:
        virtual ~comp_state_pool() = default;
        virtual void import(entt::registry &, const entity_map &) const = 0;
        virtual bool empty() const = 0;
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

                if (!registry.valid(local_entity) || !registry.all_of<Component>(local_entity)) {
                    continue;
                }

                auto comp = pair.second;
                internal::map_child_entity(registry, emap, comp);
                registry.replace<Component>(local_entity, comp);
                registry.get_or_emplace<dirty>(local_entity).template updated<Component>();
            }
        }

        bool empty() const override {
            return m_data.empty();
        }

        void insert(entt::entity entity, const Component &comp) {
            m_data.emplace_back(entity, comp);
        }

    private:
        std::vector<std::pair<entt::entity, Component>> m_data;
    };
}

class comp_state_history {
    auto first_after(double timestamp) {
        return std::find_if(history.begin(), history.end(),
                            [timestamp](auto &&elem) { return elem.timestamp > timestamp; });
    }

public:
    struct snapshot {
        std::vector<std::unique_ptr<detail::comp_state_pool>> pools;
    };

    struct element {
        snapshot snapshot;
        double timestamp;

        void import(entt::registry &registry, const entity_map &emap) const {
            for (auto &pool : snapshot.pools) {
                pool->import(registry, emap);
            }
        }
    };

protected:
    virtual snapshot take_snapshot(const entt::registry &registry, const entt::sparse_set &entities) {
        return {};
    }

    virtual snapshot take_snapshot(const packet::registry_snapshot &snap, const entt::sparse_set &entities) {
        return {};
    }

public:
    virtual ~comp_state_history() = default;

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

    template<typename Func>
    void until(double time, Func func) const {
        std::lock_guard lock(mutex);

        for (auto &elem : history) {
            if (elem.timestamp > time) {
                break;
            }

            func(elem);
        }
    }

    void import_until(double time, entt::registry &registry, const entity_map &emap) const {
        until(time, [&](auto &&elem) {
            elem.import(registry, emap);
        });
    }

    void import_each(double time, double length_of_time, entt::registry &registry, const entity_map &emap) const {
        each(time, length_of_time, [&](auto &&elem) {
            elem.import(registry, emap);
        });
    }

protected:
    std::vector<element> history;
    mutable std::mutex mutex;
};

template<typename... Components>
class comp_state_history_impl : public comp_state_history {

    template<typename Component>
    void add_to_snapshot(snapshot &snapshot, const entt::registry &registry, const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Component>) {
            auto view = registry.view<Component>();
            auto pool = std::unique_ptr<detail::comp_state_pool_impl<Component>>{};

            for (auto entity : entities) {
                if (view.contains(entity)) {
                    if (!pool) {
                        pool.reset(new detail::comp_state_pool_impl<Component>);
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

    template<typename Component>
    void add_to_snapshot(snapshot &snapshot, const std::vector<entt::entity> &pool_entities,
                         const pool_snapshot &pool_snapshot, const entt::sparse_set &entities) {
        if constexpr(!std::is_empty_v<Component>) {
            auto pool = std::unique_ptr<detail::comp_state_pool_impl<Component>>{};
            auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool_snapshot.ptr.get());

            for (size_t i = 0; i < typed_pool->entity_indices.size(); ++i) {
                auto entity_index = typed_pool->entity_indices[i];
                auto entity = pool_entities[entity_index];
                auto &comp = typed_pool->components[i];

                if (entities.contains(entity)) {
                    if (!pool) {
                        pool.reset(new detail::comp_state_pool_impl<Component>);
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
    snapshot take_snapshot(const entt::registry &registry, const entt::sparse_set &entities) override {
        snapshot snapshot;
        (add_to_snapshot<Components>(snapshot, registry, entities), ...);
        return snapshot;
    }

    snapshot take_snapshot(const packet::registry_snapshot &snap, const entt::sparse_set &entities) override {
        snapshot snapshot;
        for (auto &pool : snap.pools) {
            ((entt::type_index<Components>::value() == pool.ptr->get_type_id() ?
                add_to_snapshot<Components>(snapshot, snap.entities, pool, entities) :
                (void)0), ...);
        }
        return snapshot;
    }

public:
    comp_state_history_impl() = default;
    comp_state_history_impl([[maybe_unused]] std::tuple<Components...>) {}
};

}

#endif // EDYN_NETWORKING_COMP_STATE_HISTORY_HPP
