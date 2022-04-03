#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include <memory>
#include <variant>
#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/context/external_system.hpp"
#include "edyn/util/make_reg_op_builder.hpp"
#include "edyn/collision/should_collide.hpp"
#include "edyn/networking/settings/client_network_settings.hpp"
#include "edyn/networking/settings/server_network_settings.hpp"

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder_default();

using should_collide_func_t = decltype(&should_collide_default);

struct component_index_source;

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
    vector3 gravity {gravity_earth};

    unsigned num_solver_velocity_iterations {8};
    unsigned num_solver_position_iterations {3};
    unsigned num_restitution_iterations {8};
    unsigned num_individual_restitution_iterations {3};

    make_reg_op_builder_func_t make_reg_op_builder {&make_reg_op_builder_default};
    std::shared_ptr<component_index_source> index_source;
    external_system_func_t external_system_init {nullptr};
    external_system_func_t external_system_pre_step {nullptr};
    external_system_func_t external_system_post_step {nullptr};
    should_collide_func_t should_collide_func {&should_collide_default};

    std::variant<
        std::monostate,
        client_network_settings,
        server_network_settings> network_settings;

    settings();
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
