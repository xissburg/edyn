#ifndef EDYN_CONTEXT_STEP_CALLBACK_HPP
#define EDYN_CONTEXT_STEP_CALLBACK_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

using step_callback_t = void(*)(entt::registry &);

/**
 * @brief Assigns a function to be called before each simulation step.
 * @remark When running in sequential mode, multiple steps of a fixed time
 * length are performed in each call to `edyn::update`. This callback is
 * called before every step.
 * @remark When running in asynchronous mode this function will be called
 * in a worker thread. The simulation worker private registry will be given
 * in the sole argument.
 * @remark In a networked application, this function will be called before
 * each step during client-side extrapolation in a background thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_pre_step_callback(entt::registry &registry, step_callback_t func);

/**
 * @brief Assigns a function to be called after each simulation step.
 * @remark Same remarks in `set_pre_step_callback` apply.
 * @param registry Data source.
 * @param func The function.
 */
void set_post_step_callback(entt::registry &registry, step_callback_t func);

/**
 * @brief Remove all step callback functions.
 * @param registry Data source.
 */
void remove_pre_step_callback(entt::registry &registry);
void remove_post_step_callback(entt::registry &registry);

}

#endif // EDYN_CONTEXT_STEP_CALLBACK_HPP
