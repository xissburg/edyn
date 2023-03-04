#ifndef EDYN_CONFIG_CONSTANTS_HPP
#define EDYN_CONFIG_CONSTANTS_HPP

#include <cstdint>
#include "edyn/math/constants.hpp"

namespace edyn {

inline constexpr size_t max_contacts = 4;

/**
 * Collisions will start once the signed distance along the contact normal
 * between the closest points is below this value.
 */
inline constexpr auto collision_threshold = scalar(0.01);

/**
 * If the normal or tangential separation of a contact point goes above this
 * value, it will be destroyed.
 */
inline constexpr auto contact_breaking_threshold = scalar(0.02);

/**
 * During collision detection, as new points are added to the result, existing
 * contact points that are this close to a new point will be merged.
 */
inline constexpr auto contact_merging_threshold = scalar(0.01);

/**
 * When adding new points to a contact manifold, new contact points will be
 * merged with existing ones if their distance is below this threshold, thus
 * extending their lifetime.
 */
inline constexpr auto contact_caching_threshold = scalar(0.04);

/**
 * The magnitude of the linear and angular velocity of all rigid bodies in an
 * island must stay under these thresholds for the island to eventually fall
 * asleep.
 */
inline constexpr auto island_linear_sleep_threshold = scalar(0.005);
inline constexpr auto island_angular_sleep_threshold = pi / scalar(48);

/**
 * The amount of time in seconds that the velocity of all rigid bodies must stay
 * under the threshold for the island to fall asleep.
 */
inline constexpr auto island_time_to_sleep = scalar(2);

/**
 * Being exact when determining support features can lead to the undesired
 * feature being picked due to the limitations of floating point math. Usually,
 * the support point is found and then all points behind it within a tolerance
 * are selected as part of the support feature.
 */
inline constexpr auto support_feature_tolerance = scalar(0.005);

/**
 * Error correction rate when solving contact position constraints.
 */
inline constexpr auto contact_position_correction_rate = scalar(0.2);

/**
 * Minimum acceptable error in the contact position solver.
 */
inline constexpr auto contact_position_solver_min_error = scalar(-0.005);

/**
 * On initialization of a `convex_mesh` unique face normals and edge directions
 * are calculated to decrease the number of separating axis to be tested during
 * collision detection. If the cosine of the angle between two directions is
 * greater than one minus this value, they're considered similar and only one
 * will be used during SAT.
 */
inline constexpr auto convex_mesh_relevant_direction_tolerance = scalar(0.0006);

/**
 * Tolerance value used when validating convexity of meshes.
 */
inline constexpr auto convex_mesh_validation_parallel_tolerance = scalar(0.005);

}

#endif // EDYN_CONFIG_CONSTANTS_HPP
