#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"

namespace edyn {

class world final {
public:
    world(entt::registry& reg);
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

    void update_current_positions();

    template<auto Func>
    void defer(entt::connect_arg_t<Func> arg) {
        std::lock_guard<std::mutex> lock(mutex);
        auto d = entt::delegate(arg);
        delegates.push_back(d);
    }

    template<auto Candidate, typename Type>
    void defer(entt::connect_arg_t<Candidate> arg, Type &value_or_instance) {
        std::lock_guard<std::mutex> lock(mutex);
        auto d = entt::delegate(arg, value_or_instance);
        delegates.push_back(d);
    }


    using update_signal_func_t = void(scalar);

    entt::sink<update_signal_func_t> update_sink() {
        return {update_signal};
    }

    void on_construct_position(entt::entity, entt::registry &, position&);
    void on_construct_linvel(entt::entity, entt::registry &, linvel&);

    scalar fixed_dt {1.0/60};

private:
    entt::registry* registry;
    std::vector<entt::scoped_connection> connections;
    scalar residual_dt {0};
    std::atomic<uint64_t> step_ {0};
    std::atomic<double> local_time_;
    std::atomic_bool running {false};
    entt::sigh<update_signal_func_t> update_signal;
    std::mutex mutex;
    std::vector<entt::delegate<void(entt::registry&)>> delegates;

    void present();
    void call_delegates();
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP