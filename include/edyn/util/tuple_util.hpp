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
    template<typename T, typename... Ts, size_t... I>
    constexpr size_t index_of(std::index_sequence<I...>) {
        auto idx = std::numeric_limits<size_t>::max();
        ((idx = (std::is_same_v<std::tuple_element_t<I, std::tuple<Ts...>>, T> ? I : idx)), ...);
        return idx;
    }
}

/**
 * Find index of a type in a template parameter pack.
 */
template<typename T, typename... Ts>
constexpr size_t index_of() {
    return detail::index_of<T, Ts...>(std::make_index_sequence<sizeof...(Ts)>{});
}

/**
 * Find index of a type in a tuple.
 */
template<typename T, typename... Ts>
constexpr size_t index_of(std::tuple<Ts...>) {
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
struct tuple_cat_type;

template<typename... Ts, typename... Us>
struct tuple_cat_type<std::tuple<Ts...>, std::tuple<Us...>> {
    using type = std::tuple<Ts..., Us...>;
};

}

#endif // EDYN_UTIL_TUPLE_UTIL_HPP
