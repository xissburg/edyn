#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include <memory>
#include <variant>
#include "edyn/config/execution_mode.hpp"
#include "edyn/context/task.hpp"
#include "edyn/context/step_callback.hpp"
#include "edyn/context/start_thread.hpp"
#include "edyn/collision/should_collide.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/networking/settings/client_network_settings.hpp"
#include "edyn/networking/settings/server_network_settings.hpp"
#include "edyn/time/time.hpp"

namespace edyn {

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
    vector3 gravity {gravity_earth};

    unsigned max_steps_per_update {10};
    unsigned num_solver_velocity_iterations {8};
    unsigned num_solver_position_iterations {3};
    unsigned num_restitution_iterations {8};
    unsigned num_individual_restitution_iterations {3};

    edyn::execution_mode execution_mode;

    start_thread_func_t *start_thread_func {&start_thread_func_default};
    enqueue_task_t *enqueue_task {&enqueue_task_default};
    enqueue_task_wait_t *enqueue_task_wait {&enqueue_task_wait_default};

    init_callback_t init_callback {nullptr};
    init_callback_t deinit_callback {nullptr};
    step_callback_t pre_step_callback {nullptr};
    step_callback_t post_step_callback {nullptr};

    should_collide_func_t should_collide_func {&should_collide_default};

    using clear_actions_func_t = void(entt::registry &);
    clear_actions_func_t *clear_actions_func {nullptr};

    using time_func_t = double(void);
    time_func_t *time_func {&performance_time};

    std::variant<
        std::monostate,
        client_network_settings,
        server_network_settings> network_settings;
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
