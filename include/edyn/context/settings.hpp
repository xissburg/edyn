#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include <memory>
#include "edyn/math/scalar.hpp"
#include "edyn/context/external_system.hpp"
#include "edyn/parallel/make_island_delta_builder.hpp"

namespace edyn {

std::unique_ptr<island_delta_builder> make_island_delta_builder_default();

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
    make_island_delta_builder_func_t make_island_delta_builder {&make_island_delta_builder_default};
    external_system_func_t external_system_init {nullptr};
    external_system_func_t external_system_pre_step {nullptr};
    external_system_func_t external_system_post_step {nullptr};
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
