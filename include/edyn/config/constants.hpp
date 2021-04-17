#ifndef EDYN_CONFIG_CONSTANTS_HPP
#define EDYN_CONFIG_CONSTANTS_HPP

#include <cstdint>
#include "edyn/math/constants.hpp"

namespace edyn {

inline constexpr size_t max_constraint_rows = 16;
inline constexpr size_t max_contacts = 4;
inline constexpr size_t max_constrained_entities = 2;

inline constexpr scalar contact_breaking_threshold = 0.02;
inline constexpr scalar contact_caching_threshold = 0.04;
inline constexpr scalar island_time_to_sleep = 2;
inline constexpr scalar island_linear_sleep_threshold = 0.005;
inline constexpr scalar island_angular_sleep_threshold = pi / 48;
inline constexpr scalar support_polygon_tolerance = 0.004;

}

#endif // EDYN_CONFIG_CONSTANTS_HPP
