#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <entt/entt.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/comp/constraint.hpp"

namespace edyn {

class world final {
public:
    world(entt::registry &);
    ~world();

    void update(scalar dt);

    void step(scalar dt);

    uint64_t current_step() const {
        return step_;
    }

    double local_time() const {
        return local_time_;
    }

    void run();

    void quit();

    using update_signal_func_t = void(scalar);

    entt::sink<update_signal_func_t> update_sink() {
        return {update_signal};
    }

    scalar fixed_dt {1.0/60};

    void on_construct_constraint(entt::entity, entt::registry &, constraint &);
    void on_destroy_constraint(entt::entity, entt::registry &);

private:
    entt::registry* registry;
    std::vector<entt::scoped_connection> connections;
    scalar residual_dt {0};
    std::atomic<uint64_t> step_ {0};
    std::atomic<double> local_time_;
    std::atomic_bool running {false};
    entt::sigh<update_signal_func_t> update_signal;
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP