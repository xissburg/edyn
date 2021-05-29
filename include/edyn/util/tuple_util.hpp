#ifndef EDYN_UTIL_TUPLE_UTIL_HPP
#define EDYN_UTIL_TUPLE_UTIL_HPP

#include <tuple>
#include <variant>

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

// Do the same for variants. TODO: maybe move this somewhere else.
template <typename T, typename... Us>
struct has_type<T, std::variant<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

namespace detail {
    template<typename IndexType, typename T, typename... Ts>
    struct index_of;

    template<typename IndexType, typename T, typename... Ts>
    struct index_of<IndexType, T, T, Ts...> : std::integral_constant<IndexType, 0>{};

    template<typename IndexType, typename T, typename U, typename... Ts>
    struct index_of<IndexType, T, U, Ts...> : std::integral_constant<IndexType, 1 + index_of<IndexType, T, Ts...>::value>{};

    template<typename IndexType, typename T, typename... Ts>
    static constexpr IndexType index_of_v = index_of<IndexType, T, Ts...>::value;
}

/**
 * Find index of a type in a template parameter pack.
 */
template<typename T, typename... Ts, typename IndexType = size_t>
constexpr IndexType index_of() {
    return detail::index_of_v<IndexType, T, Ts...>;
}

/**
 * Find index of a type in a tuple.
 */
template<typename T, typename... Ts, typename IndexType = size_t>
constexpr IndexType index_of(std::tuple<Ts...>) {
    return index_of<T, Ts...>();
}

/**
 * Map a `std::tuple<Us...>` to `std::tuple<T<Us>...>`.
 */
template<template<typename> class T, typename U>
struct map_tuple;

template<template<typename> class T, typename... Us>
struct map_tuple<T, std::tuple<Us...>> {
    using type = std::tuple<T<Us>...>;
};

/**
 * Convert a tuple type to a variant type with the same template parameters.
 */
template<typename... Ts>
struct tuple_to_variant;

template<typename... Ts>
struct tuple_to_variant<std::tuple<Ts...>> {
    using type = std::variant<Ts...>;
};

/**
 * Concatenate tuple types.
 */
template<typename... Ts>
struct tuple_type_cat;

template<typename... Ts, typename... Us>
struct tuple_type_cat<std::tuple<Ts...>, std::tuple<Us...>> {
    using type = std::tuple<Ts..., Us...>;
};

template<typename... Ts, typename... Us, typename... Vs>
struct tuple_type_cat<std::tuple<Ts...>, std::tuple<Us...>, std::tuple<Vs...>> {
    using type = std::tuple<Ts..., Us..., Vs...>;
};

}

#endif // EDYN_UTIL_TUPLE_UTIL_HPP
