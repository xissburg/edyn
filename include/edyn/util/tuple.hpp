#ifndef EDYN_UTIL_TUPLE_HPP
#define EDYN_UTIL_TUPLE_HPP

#include <tuple>

namespace edyn {

/**
 * Check if a tuple contains a given type. Usage:
 * `if constexpr(has_type<T, a_tuple>::value) { ... }`
 * Reference: https://stackoverflow.com/a/41171291
 */
template <typename T, typename Tuple>
struct has_type;

template <typename T, typename... Us>
struct has_type<T, std::tuple<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

}

#endif // EDYN_UTIL_TUPLE_HPP