#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include "edyn/math/scalar.hpp"
#include <entt/fwd.hpp>

namespace edyn {

using external_system_func_t = void(*)(entt::registry &);

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
    external_system_func_t external_system_init {nullptr};
    external_system_func_t external_system_pre_step {nullptr};
    external_system_func_t external_system_post_step {nullptr};
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
