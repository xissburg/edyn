#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <entt/entt.hpp>

#include "solver.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/collision/simple_broadphase.hpp"

#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/island_worker.hpp"

namespace edyn {

struct island_info {
    island_worker_context_base *m_worker;
    message_queue_in_out m_message_queue;

    island_info(island_worker_context_base *worker,
                message_queue_in_out message_queue)
        : m_worker(worker)
        , m_message_queue(message_queue)
    {}
};

class world final {
public:
    world(entt::registry &);
    ~world();

    void update(scalar dt);

    uint64_t current_step() const {
        return step_;
    }

    double local_time() const {
        return local_time_;
    }

    void run();

    void quit();

    void set_paused(bool);
    void step();

    using registry_snapshot_type = decltype(registry_snapshot(all_components{}));
    using update_signal_func_t = void(scalar);
    using step_signal_func_t = void(uint64_t);

    entt::sink<update_signal_func_t> update_sink() {
        return {update_signal};
    }

    /* broadphase &get_broaphase() {
        return bphase;
    } */

    template<typename... Component>
    void refresh(entt::entity entity) {
        auto snapshot = registry_snapshot(all_components{});
        (snapshot.updated(entity, registry->get<Component>(entity)), ...);
        auto &node = registry->get<island_node>(entity);

        for (auto island_entity : node.island_entities) {
            auto &info = m_island_info_map.at(island_entity);
            info.m_message_queue.send<registry_snapshot_type>(snapshot);
        }
    }

    void refresh_all(entt::entity);
    void on_broadphase_intersect(entt::entity, entt::entity);
    void on_construct_relation(entt::entity, entt::registry &, relation &);
    void on_destroy_relation(entt::entity, entt::registry &);
    void merge_entities(entt::entity, entt::entity, entt::entity rel_entity);

    scalar fixed_dt {1.0/60};
    solver sol;
    std::unordered_map<entt::entity, island_info> m_island_info_map;

private:
    entt::registry* registry;
    simple_broadphase bphase;
    std::vector<entt::scoped_connection> connections;

    scalar residual_dt {0};
    std::atomic<uint64_t> step_ {0};
    std::atomic<double> local_time_;
    std::atomic_bool running {false};
    bool m_paused {false};
    entt::sigh<update_signal_func_t> update_signal;
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP