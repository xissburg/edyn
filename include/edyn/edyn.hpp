#ifndef EDYN_EDYN_HPP
#define EDYN_EDYN_HPP

#include "edyn/build_settings.h"
#include "edyn/config/execution_mode.hpp"
#include "edyn/config/solver_iteration_config.hpp"
#include "math/constants.hpp"
#include "math/scalar.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "math/quaternion.hpp"
#include "math/matrix3x3.hpp"
#include "math/transform.hpp"
#include "math/math.hpp"
#include "time/time.hpp"
#include "util/rigidbody.hpp"
#include "util/constraint_util.hpp"
#include "util/exclude_collision.hpp"
#include "util/gravity_util.hpp"
#include "util/insert_material_mixing.hpp"
#include "collision/contact_signal.hpp"
#include "context/step_callback.hpp"
#include "collision/raycast.hpp"
#include "shapes/shapes.hpp"
#include "comp/shared_comp.hpp"
#include "comp/present_position.hpp"
#include "comp/present_orientation.hpp"
#include "constraints/constraint.hpp"
#include "serialization/s11n.hpp"

namespace edyn {

/**
 * @brief Initialization configuration.
 */
struct init_config {
    // Number of worker threads to spawn. If zero, value will be taken from
    // `std::thread::hardware_concurrency`.
    size_t num_worker_threads {0};
    edyn::execution_mode execution_mode {edyn::execution_mode::asynchronous};
};

/**
 * @brief Attaches Edyn to an EnTT registry.
 * @param registry The registry to be setup to run Edyn.
 */
void attach(entt::registry &registry, const init_config &config = {});

/**
 * @brief Detaches Edyn from an EnTT registry.
 * @param registry The registry to be freed from Edyn's context.
 */
void detach(entt::registry &registry);

/**
 * @brief Get the fixed simulation delta time for each step.
 * @param registry Data source.
 * @return Fixed delta time in seconds.
 */
scalar get_fixed_dt(const entt::registry &registry);

/**
 * @brief Set the fixed simulation delta time for each step.
 * @param registry Data source.
 * @param dt Delta time in seconds.
 */
void set_fixed_dt(entt::registry &registry, scalar dt);

/**
 * @brief Checks if simulation is paused.
 * @param registry Data source.
 * @return Whether simulation is paused.
 */
bool is_paused(const entt::registry &registry);

/**
 * @brief Pauses simulation.
 * @param registry Data source.
 */
void set_paused(entt::registry &registry, bool paused);

/**
 * @brief Updates the simulation. Call it regularly.
 * The actual physics simulation runs in other threads. This function only
 * does coordination of background simulation jobs. It's expected to be a
 * lightweight call.
 * @param registry Data source.
 */
void update(entt::registry &registry);

/**
 * @brief Runs a single step for a paused simulation.
 * @param registry Data source.
 */
void step_simulation(entt::registry &registry);

execution_mode get_execution_mode(const entt::registry &registry);

}

#endif // EDYN_EDYN_HPP
