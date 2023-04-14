#ifndef EDYN_CONTEXT_STEP_CALLBACK_HPP
#define EDYN_CONTEXT_STEP_CALLBACK_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

using step_callback_t = void(*)(entt::registry &);
using init_callback_t = void(*)(entt::registry &);
struct settings;

namespace internal {
    void notify_settings(entt::registry &registry, const edyn::settings &settings);
}

/**
 * @brief Assign a function to be called before each simulation step.
 * @remark When running in sequential mode, multiple steps of a fixed time
 * length are performed in each call to `edyn::update`. This callback is
 * called before every step.
 * @remark When running in asynchronous mode this function will be called
 * in a worker thread. The simulation worker private registry will be given
 * in the sole argument.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_pre_step_callback(entt::registry &registry, step_callback_t func);

/**
 * @brief Assign a function to be called after each simulation step.
 * @remark Same remarks in `set_pre_step_callback` apply.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_post_step_callback(entt::registry &registry, step_callback_t func);

/**
 * @brief Assign a function to be called when the simulation worker starts.
 * This callback can prepare the worker's registry by assigning context
 * variables that will be needed in the pre-step callback, for example.
 * Only relevant if running in asynchronous mode.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_init_callback(entt::registry &registry, init_callback_t func);

/**
 * @brief Assign a function to be called when the simulation worker finishes
 * running. Only relevant if running in asynchronous mode.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_deinit_callback(entt::registry &registry, init_callback_t func);

}

#endif // EDYN_CONTEXT_STEP_CALLBACK_HPP
