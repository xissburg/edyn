#ifndef EDYN_UTIL_VISIT_COMPONENT_HPP
#define EDYN_UTIL_VISIT_COMPONENT_HPP

#include <entt/fwd.hpp>
#include <array>

namespace edyn {

/**
 * @brief Creates a function that invokes a visitor with a component argument.
 * @tparam T Component type.
 * @tparam ViewType Type of view that holds components of type T among others.
 * @tparam VisitorType Type of visitor function which takes an auto rvalue.
 * @return A function that takes an entity, a view and a visitor and calls the
 * visitor with the component for the entity that's in the view.
 */
template<typename T, typename ViewType, typename VisitorType>
constexpr auto make_visit_function() {
    return [] (entt::entity entity, ViewType &view, VisitorType visitor) {
        visitor(view.template get<T>(entity));
    };
}

/**
 * @brief Stores an array of visiting functions for each given type.
 */
template<typename VisitorType, typename ViewType, typename... Ts>
struct visit_function_array {
    using VisitFuncType = void(*)(entt::entity, ViewType &, VisitorType);
    std::array<VisitFuncType, sizeof...(Ts)> functions;

    constexpr visit_function_array() : functions{} {
        size_t i = 0;
        ((functions[i++] = make_visit_function<Ts, ViewType, VisitorType>()), ...);
    }
};

/**
 * @brief Holds a table of functions to visit each given component type.
 */
template<typename VisitorType, typename ViewType, typename... Ts>
struct visitor_table {
    static constexpr auto array = visit_function_array<VisitorType, ViewType, Ts...>();
};

/**
 * @brief Invokes visitor with the component of type at the index in the list
 * of provided component types in constant time.
 * @tparam Ts Family of related component types.
 * @tparam IndexType Type of integral index.
 * @tparam ViewType Type of view that holds components of all types `Ts...`.
 * @tparam VisitorType Type of visitor function which takes an auto rvalue.
 * @param index Index in the family of component types of which component should
 * be visited among all types.
 * @param entity Entity of interest.
 * @param view The view where the component will be fetched from.
 * @param visitor Function that will be called with the desired component.
 */
template<typename... Ts, typename IndexType, typename ViewType, typename VisitorType>
void visit_component(IndexType index, entt::entity entity,
                     ViewType &view, VisitorType visitor) {
    constexpr auto table = visitor_table<VisitorType, ViewType, Ts...>{};
    table.array.functions[index](entity, view, visitor);
}

template<typename... Ts, typename IndexType, typename ViewType, typename VisitorType>
void visit_component(std::tuple<Ts...>, IndexType index, entt::entity entity,
                     ViewType &view, VisitorType visitor) {
    visit_component<Ts...>(index, entity, view, visitor);
}

}

#endif // EDYN_UTIL_VISIT_COMPONENT_HPP
