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

/**
 * Find index of a type in a template parameter pack.
 */
template<typename T, typename... Ts>
struct index_of;

template<typename T, typename... Ts>
struct index_of<T, T, Ts...> : std::integral_constant<size_t, 0>{};

template<typename T, typename U, typename... Ts>
struct index_of<T, U, Ts...> : std::integral_constant<size_t, 1 + index_of<T, Ts...>::value>{};

template<typename T, typename... Ts>
static constexpr size_t index_of_v = index_of<T, Ts...>::value;

}

#endif // EDYN_UTIL_TUPLE_HPP