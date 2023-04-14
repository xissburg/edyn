#ifndef EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_CALLBACK_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_CALLBACK_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

using extrapolation_callback_t = void(*)(entt::registry &);

/**
 * @brief Assign a function to be called when the extrapolation worker starts
 * running in a dedicated thread.
 * @param registry Data source in the extrapolator.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_init_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign a function to be called when the extrapolation worker finishes
 * running.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_deinit_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign a function to be called before each extrapolation job begins.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_begin_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign a function to be called after each extrapolation job ends.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_finish_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign a function to be called before each simulation step during
 * extrapolation. Similar to `set_pre_step_callback` though a separate callback
 * is allowed because it might be advantageous to run a different and simpler
 * logic during extrapolation.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_pre_step_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign a function to be called after each simulation step during
 * extrapolation. Similar to `set_post_step_callback` though a separate callback
 * is allowed because it might be advantageous to run a different and simpler
 * logic during extrapolation.
 * @param registry Data source.
 * @param func The function. Pass `nullptr` to unassign.
 */
void set_extrapolation_post_step_callback(entt::registry &registry, extrapolation_callback_t func);

/**
 * @brief Assign all extrapolation callbacks in one call.
 * @param registry Data source.
 * @param init Init callback.
 * @param deinit Deinit callback.
 * @param begin Begin callback.
 * @param finish Finish callback.
 * @param pre_step Pre-step callback.
 * @param post_step Post-step callback.
 */
void set_extrapolation_callbacks(entt::registry &registry,
                                 extrapolation_callback_t init,
                                 extrapolation_callback_t deinit,
                                 extrapolation_callback_t begin,
                                 extrapolation_callback_t finish,
                                 extrapolation_callback_t pre_step,
                                 extrapolation_callback_t post_step);

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_CALLBACK_HPP
