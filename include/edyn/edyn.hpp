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
#include "context/start_thread.hpp"
#include "context/task.hpp"
#include "collision/raycast.hpp"
#include "shapes/shapes.hpp"
#include "comp/shared_comp.hpp"
#include "comp/present_position.hpp"
#include "comp/present_orientation.hpp"
#include "constraints/constraint.hpp"
#include "serialization/s11n.hpp"
#include "replication/register_external.hpp"
#include <optional>

namespace edyn {

/**
 * @brief Initialization configuration.
 */
struct init_config {
    // Number of worker threads to spawn. If zero, value will be taken from
    // `std::thread::hardware_concurrency`.
    size_t num_worker_threads {0};
    scalar fixed_dt {scalar(1.0 / 60)};
    edyn::execution_mode execution_mode {edyn::execution_mode::asynchronous};
    // If using a custom time source, assign the current time here for the
    // engine initialization.
    std::optional<double> timestamp;
    // Function to create threads which uses `std::thread` by default. Replace
    // to use alternative threads.
    // When running in async mode, Edyn needs to start a simulation thread.
    // When running as a network client (i.e. `init_network_client` is called),
    // Edyn needs to start an extrapolation thread to run latency compensation
    // in parallel without freezing the simulation.
    start_thread_func_t *start_thread_func {&start_thread_func_default};
    // Function to run a task on worker threads. Must return immediately after
    // scheduling tasks.
    enqueue_task_t *enqueue_task {&enqueue_task_default};
    // Function to run a task on worker threads and return after the work is done.
    enqueue_task_wait_t *enqueue_task_wait {&enqueue_task_wait_default};
};

/**
 * @brief Attaches Edyn to an EnTT registry.
 * @param registry The registry to be setup to run Edyn.
 */
void attach(entt::registry &registry, const init_config &config = {});

/**
 * @brief Detaches Edyn from an EnTT registry.
 * @remark It will cleanup existing rigid body and constraint entities but will
 * not destroy them.
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
 * @brief The time that has passed since the last update determines how many
 * steps with a fixed delta time will be performed in the current update.
 * If too much time has passed, too many steps will be ran, which could cause
 * slow downs. For that reason, the number of steps per update is capped.
 * @param registry Data source.
 * @param max_steps Maximum number of steps per update.
 */
void set_max_steps_per_update(entt::registry &registry, unsigned);

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
 * @brief Steps the simulation forward. Call it regularly.
 * @param registry Data source.
 */
void update(entt::registry &registry);

/**
 * @brief Same as `edyn::update(entt::registry &)` but it takes a timestamp
 * parameter from an external time source.
 * @remark Using a single and consistent timestamp that is sampled at the very
 * beginning of the game loop is recommended to avoid small glitches the
 * presentation transforms (`edyn::present_position` and
 * `edyn::present_orientation`).
 * @param registry Data source.
 * @param time The current time.
 */
void update(entt::registry &registry, double time);

/**
 * @brief Runs a single step for a paused simulation.
 * @param registry Data source.
 */
void step_simulation(entt::registry &registry);

/**
 * @brief Runs a single step for a paused simulation when using a custom
 * time source.
 * @param registry Data source.
 * @param time The current time.
 */
void step_simulation(entt::registry &registry, double time);

execution_mode get_execution_mode(const entt::registry &registry);

/**
 * @brief Assign a custom time source function to be used by the engine
 * internally. The same time source must be used to generate a timestamp
 * when calling any function that needs one in their arguments, such as
 * `edyn::update` and `edyn::step_simulation`.
 * @remark The default time source is `edyn::performance_time`.
 * @param registry Data source.
 * @param time_func Pointer to a function that returns a timestamp in seconds.
 * Must be thread-safe.
 */
void set_time_source(entt::registry &registry, double(*time_func)(void));

/**
 * @brief Get timestamp from time source.
 * @param registry Data source.
 * @return Current time.
 */
double get_time(entt::registry &registry);

/**
 * @brief Pointer to function that runs tasks in worker threads.
 * @param registry Data source.
 * @return Function pointer.
 */
enqueue_task_t * get_enqueue_task(entt::registry &registry);

/**
 * @brief Pointer to function that runs tasks in worker threads and waits for
 * their execution to finish.
 * @param registry Data source.
 * @return Function pointer.
 */
enqueue_task_wait_t * get_enqueue_task_wait(entt::registry &registry);

}

#endif // EDYN_EDYN_HPP
