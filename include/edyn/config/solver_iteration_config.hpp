#ifndef EDYN_CONFIG_SOLVER_ITERATION_CONFIG_HPP
#define EDYN_CONFIG_SOLVER_ITERATION_CONFIG_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Get the number of constraint solver velocity iterations.
 * @param registry Data source.
 * @return Number of solver velocity iterations.
 */
unsigned get_solver_velocity_iterations(const entt::registry &registry);

/**
 * @brief Set the number of constraint solver velocity iterations.
 * @param registry Data source.
 * @param iterations Number of solver velocity iterations.
 */
void set_solver_velocity_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of constraint solver position iterations.
 * @param registry Data source.
 * @return Number of solver position iterations.
 */
unsigned get_solver_position_iterations(const entt::registry &registry);

/**
 * @brief Set the number of constraint solver position iterations.
 * @param registry Data source.
 * @param iterations Number of solver position iterations.
 */
void set_solver_position_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of restitution iterations.
 * @param registry Data source.
 * @return Number of restitution iterations.
 */
unsigned get_solver_restitution_iterations(const entt::registry &registry);

/**
 * @brief Set the number of restitution iterations. The restitution solver will
 * stop early once the penetration velocity of all contact points is above a
 * threshold.
 * @param registry Data source.
 * @param iterations Number of restitution iterations.
 */
void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of individual restitution iterations.
 * @param registry Data source.
 * @return Number of individual restitution iterations.
 */
unsigned get_solver_individual_restitution_iterations(const entt::registry &registry);

/**
 * @brief Set the number of individual restitution iterations. This is the number
 * of iterations used while solving each subset of contact constraints in each
 * iteration of the restitution solver.
 * @param registry Data source.
 * @param iterations Number of individual restitution iterations.
 */
void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations);

}

#endif // EDYN_CONFIG_SOLVER_ITERATION_CONFIG_HPP
