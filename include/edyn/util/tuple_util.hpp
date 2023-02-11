#ifndef EDYN_UTIL_TUPLE_UTIL_HPP
#define EDYN_UTIL_TUPLE_UTIL_HPP

#include <tuple>
#include <array>
#include <utility>
#include <variant>

namespace edyn {

/**
 * Check if a pack contains a given type.
 */
template <typename T, typename... Us>
struct has_type : std::disjunction<std::is_same<T, Us>...> {};

template <typename T, typename Tuple>
struct tuple_has_type;

template <typename T, typename... Us>
struct tuple_has_type<T, std::tuple<Us...>> : has_type<T, Us...> {};

template <typename T, typename Variant>
struct variant_has_type;

template <typename T, typename... Us>
struct variant_has_type<T, std::variant<Us...>> : has_type<T, Us...> {};

/**
 * Get index of type in pack.
 */
template<typename IndexType, typename T, typename... Us>
struct index_of {
private:
    template<IndexType... Indices>
    static constexpr IndexType index_of_func(std::integer_sequence<IndexType, Indices...>) {
        static_assert(has_type<T, Us...>::value);
        return ((std::is_same_v<T, Us> ? Indices : 0) + ...);
    }

public:
    static constexpr IndexType value = []() {
        constexpr auto indices = std::make_integer_sequence<IndexType, sizeof...(Us)>{};
        return index_of_func(indices);
    }();
};

template<typename IndexType, typename T, typename... Ts>
static constexpr IndexType index_of_v = index_of<IndexType, T, Ts...>::value;

/**
 * Find index of a type in a tuple type.
 */
template<typename IndexType, typename T, typename Tuple>
struct tuple_type_index_of;

template<typename IndexType, typename T, typename... Ts>
struct tuple_type_index_of<IndexType, T, std::tuple<Ts...>> : index_of<IndexType, T, Ts...> {};

/**
 * Find index of a type in a tuple.
 */
template<typename IndexType, typename T, typename... Ts>
constexpr IndexType tuple_index_of(const std::tuple<Ts...> &) {
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
    template<typename VisitorType, typename TupleType, std::size_t Index>
    constexpr auto make_visit_tuple_function() {
        return [](TupleType &tuple, VisitorType visitor) {
            visitor(std::get<Index>(tuple));
        };
    }

    template<typename VisitorType, typename TupleType, std::size_t... Indices>
    constexpr auto make_visit_tuple_functions(std::index_sequence<Indices...>) {
        using VisitFuncType = void(*)(TupleType &, VisitorType);
        return std::array<VisitFuncType, sizeof...(Indices)>{
            {make_visit_tuple_function<VisitorType, TupleType, Indices>()...}
        };
    }

    template<typename VisitorType, typename... Ts>
    struct visit_tuple_function_array {
        using TupleType = std::tuple<Ts...>;
        using VisitFuncType = void(*)(TupleType &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_tuple_function_array()
            : functions(make_visit_tuple_functions<VisitorType, TupleType>(std::make_index_sequence<sizeof...(Ts)>{}))
        {}
    };

    template<typename VisitorType, typename... Ts>
    struct tuple_visitor_table {
        static constexpr auto array = visit_tuple_function_array<VisitorType, Ts...>();
    };

    // Const version.
    template<typename VisitorType, typename TupleType, std::size_t Index>
    constexpr auto make_visit_const_tuple_function() {
        return [](const TupleType &tuple, VisitorType visitor) {
            visitor(std::get<Index>(tuple));
        };
    }

    template<typename VisitorType, typename TupleType, std::size_t... Indices>
    constexpr auto make_visit_const_tuple_functions(std::index_sequence<Indices...>) {
        using VisitFuncType = void(*)(const TupleType &, VisitorType);
        return std::array<VisitFuncType, sizeof...(Indices)>{
            {make_visit_const_tuple_function<VisitorType, TupleType, Indices>()...}
        };
    }

    template<typename VisitorType, typename... Ts>
    struct visit_const_tuple_function_array {
        using TupleType = std::tuple<Ts...>;
        using VisitFuncType = void(*)(const TupleType &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_const_tuple_function_array()
            : functions(make_visit_const_tuple_functions<VisitorType, TupleType>(std::make_index_sequence<sizeof...(Ts)>{}))
        {}
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
