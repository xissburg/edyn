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
#include "edyn/parallel/island_coordinator.hpp"

namespace edyn {

class world;

class world final {

    template<typename Message, typename... Args>
    void send_message(entt::entity island_entity, Args &&... args) {
        m_island_info_map.at(island_entity).m_message_queue.send<Message>(std::forward<Args>(args)...);
    }

public:
    world(entt::registry &);
    ~world();

    void update(scalar dt);

    void run();

    void quit();

    void set_paused(bool);
    void step();

    using update_signal_func_t = void(scalar);
    using step_signal_func_t = void(uint64_t);

    entt::sink<update_signal_func_t> update_sink() {
        return {m_update_signal};
    }

    void on_broadphase_intersect(entt::entity, entt::entity);

    scalar m_fixed_dt {1.0/60};
    solver m_solver;

private:
    entt::registry* m_registry;
    simple_broadphase m_bphase;
    island_coordinator m_island_coordinator;
    std::vector<entt::scoped_connection> m_connections;

    std::atomic_bool m_running {false};
    bool m_paused {false};
    entt::sigh<update_signal_func_t> m_update_signal;
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP