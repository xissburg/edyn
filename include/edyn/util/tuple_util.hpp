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

template<typename IndexType, typename T, typename... Ts>
struct index_of;

template<typename IndexType, typename T, typename... Ts>
struct index_of<IndexType, T, T, Ts...> : std::integral_constant<IndexType, 0>{};

template<typename IndexType, typename T, typename U, typename... Ts>
struct index_of<IndexType, T, U, Ts...> : std::integral_constant<IndexType, 1 + index_of<IndexType, T, Ts...>::value>{};

template<typename IndexType, typename T, typename... Ts>
static constexpr IndexType index_of_v = index_of<IndexType, T, Ts...>::value;

/**
 * Find index of a type in a tuple type.
 */
template<typename IndexType, typename T, typename... Ts>
struct index_of<IndexType, T, std::tuple<Ts...>> : index_of<IndexType, T, Ts...>{};

/**
 * Find index of a type in a tuple.
 */
template<typename T, typename IndexType = size_t, typename... Ts>
constexpr IndexType tuple_index_of(std::tuple<Ts...>) {
    return index_of_v<IndexType, T, Ts...>;
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
 * Convert a tuple to a variant with the same types.
 */
template<typename... Ts>
constexpr auto tuple_to_variant(std::tuple<Ts...>) {
    return std::variant<Ts...>{};
}

namespace detail {
    template<typename VisitorType, typename TupleType, typename T>
    constexpr auto make_visit_tuple_function() {
        return [] (TupleType &tuple, VisitorType visitor) {
            visitor(std::get<T>(tuple));
        };
    }

    template<typename VisitorType, typename... Ts>
    struct visit_tuple_function_array {
        using TupleType = std::tuple<Ts...>;
        using VisitFuncType = void(*)(TupleType &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_tuple_function_array() : functions{} {
            size_t i = 0;
            ((functions[i++] = make_visit_tuple_function<VisitorType, TupleType, Ts>()), ...);
        }
    };

    template<typename VisitorType, typename... Ts>
    struct tuple_visitor_table {
        static constexpr auto array = visit_tuple_function_array<VisitorType, Ts...>();
    };

    template<typename VisitorType, typename TupleType, typename T>
    constexpr auto make_visit_const_tuple_function() {
        return [] (const TupleType &tuple, VisitorType visitor) {
            visitor(std::get<T>(tuple));
        };
    }

    template<typename VisitorType, typename... Ts>
    struct visit_const_tuple_function_array {
        using TupleType = std::tuple<Ts...>;
        using VisitFuncType = void(*)(const TupleType &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_const_tuple_function_array() : functions{} {
            size_t i = 0;
            ((functions[i++] = make_visit_const_tuple_function<VisitorType, TupleType, Ts>()), ...);
        }
    };

    template<typename VisitorType, typename... Ts>
    struct const_tuple_visitor_table {
        static constexpr auto array = visit_const_tuple_function_array<VisitorType, Ts...>();
    };
}

/**
 * Allows access to a tuple element via a runtime index.
 * @tparam Ts Tuple element types.
 * @tparam IndexType Type of index.
 * @tparam VisitorType Type of visitor function.
 * @param tuple Tuple to be visited.
 * @param index Index of element in tuple.
 * @param visitor Visitor function.
 */
template<typename... Ts, typename IndexType, typename VisitorType>
void visit_tuple(std::tuple<Ts...> &tuple, IndexType index, VisitorType visitor) {
    constexpr auto table = detail::tuple_visitor_table<VisitorType, Ts...>{};
    table.array.functions[index](tuple, visitor);
}

template<typename... Ts, typename IndexType, typename VisitorType>
void visit_tuple(const std::tuple<Ts...> &tuple, IndexType index, VisitorType visitor) {
    constexpr auto table = detail::const_tuple_visitor_table<VisitorType, Ts...>{};
    table.array.functions[index](tuple, visitor);
}

}

#endif // EDYN_UTIL_TUPLE_UTIL_HPP
