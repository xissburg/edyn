#ifndef EDYN_CONFIG_CONSTANTS_HPP
#define EDYN_CONFIG_CONSTANTS_HPP

#include <cstdint>
#include "edyn/math/constants.hpp"

namespace edyn {

inline constexpr size_t max_constraint_rows = 16;
inline constexpr size_t max_contacts = 4;
inline constexpr size_t max_constrained_entities = 2;

/**
 * If the normal or tangential separation of a contact point goes above this
 * value, it will be destroyed. This is also the minimum distance between new
 * contact points for them to be considered.
 */
inline constexpr auto contact_breaking_threshold = scalar(0.02);

/**
 * New contact points will be merged with existing ones if their distance is
 * below this threshold, thus extending their lifetime.
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
inline constexpr auto support_feature_tolerance = scalar(0.01);

}

#endif // EDYN_CONFIG_CONSTANTS_HPP
