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
template<typename IndexType, typename T, typename... Ts>
struct index_of;

template<typename IndexType, typename T, typename... Ts>
struct index_of<IndexType, T, T, Ts...> : std::integral_constant<IndexType, IndexType{0}>{};

template<typename IndexType, typename T, typename U, typename... Ts>
struct index_of<IndexType, T, U, Ts...> : std::integral_constant<IndexType, IndexType{1} + index_of<IndexType, T, Ts...>::value>{};

template<typename IndexType, typename T, typename... Ts>
static constexpr auto index_of_v = index_of<IndexType, T, Ts...>::value;

}

#endif // EDYN_UTIL_TUPLE_HPP