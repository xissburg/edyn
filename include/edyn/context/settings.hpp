#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include <memory>
#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/context/external_system.hpp"
#include "edyn/parallel/make_island_delta_builder.hpp"
#include "edyn/collision/should_collide.hpp"

namespace edyn {

std::unique_ptr<island_delta_builder> make_island_delta_builder_default();

using should_collide_func_t = decltype(&should_collide_default);

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
    vector3 gravity {gravity_earth};
    unsigned num_solver_iterations {10};
    make_island_delta_builder_func_t make_island_delta_builder {&make_island_delta_builder_default};
    external_system_func_t external_system_init {nullptr};
    external_system_func_t external_system_pre_step {nullptr};
    external_system_func_t external_system_post_step {nullptr};
    should_collide_func_t should_collide_func {&should_collide_default};
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
