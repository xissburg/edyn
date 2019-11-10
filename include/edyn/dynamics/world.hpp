#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <entt/entt.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

class world final {
public:
    world(entt::registry& reg) :
        registry(&reg)
    {}

    void update(scalar dt);

    void step(scalar dt);

    uint64_t current_step() const {
        return step_;
    }

    void run();

    void quit();

    scalar fixed_dt {1.0/60};

    using update_signal_func_t = void(scalar);

    entt::sink<update_signal_func_t> update_sink() {
        return {update_signal};
    }

private:
    entt::registry* registry;
    scalar residual_dt {0};
    uint64_t step_ {0};
    bool running {false};
    entt::sigh<update_signal_func_t> update_signal;
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP