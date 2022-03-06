#ifndef EDYN_UTIL_VISIT_COMPONENT_HPP
#define EDYN_UTIL_VISIT_COMPONENT_HPP

#include <entt/entity/registry.hpp>
#include <array>

namespace edyn {

namespace detail {
    /**
    * @brief Creates a function that invokes a visitor with a component argument.
    * @tparam VisitorType Type of visitor function which takes an auto rvalue.
    * @tparam ViewsTupleType Type of tuple that holds a view of each of the
    * supported component types.
    * @tparam T Component type.
    * @return A function that takes an entity, a view and a visitor and calls the
    * visitor with the component for the entity that's in the view.
    */
    template<typename VisitorType, typename ViewsTupleType, typename T>
    constexpr auto make_visit_function() {
        using ViewType = entt::basic_view<entt::entity, entt::get_t<T>, entt::exclude_t<>>;
        return [] (entt::entity entity, const ViewsTupleType &views_tuple, VisitorType visitor) {
            visitor(std::get<ViewType>(views_tuple).template get<T>(entity));
        };
    }

    /**
    * @brief Stores an array of visiting functions for each given type.
    */
    template<typename VisitorType, typename... Ts>
    struct visit_function_array {
        using ViewsTupleType = std::tuple<entt::basic_view<entt::entity, entt::get_t<Ts>, entt::exclude_t<>>...>;
        using VisitFuncType = void(*)(entt::entity, const ViewsTupleType &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_function_array() : functions{} {
            size_t i = 0;
            ((functions[i++] = make_visit_function<VisitorType, ViewsTupleType, Ts>()), ...);
        }
    };

    /**
    * @brief Holds a table of functions to visit each given component type.
    */
    template<typename VisitorType, typename... Ts>
    struct visitor_table {
        static constexpr auto array = visit_function_array<VisitorType, Ts...>();
    };
}

/**
 * @brief Invokes visitor with the component of type at the index in the list
 * of provided component types in constant time.
 * @tparam Ts Family of related component types.
 * @tparam IndexType Type of integral index.
 * @tparam VisitorType Type of visitor function which takes an auto rvalue.
 * @param index Index in the family of component types of which component should
 * be visited among all types.
 * @param entity Entity of interest.
 * @param views_tuple Tuple of views for each type `Ts...`.
 * @param visitor Function that will be called with the desired component.
 */
template<typename... Ts, typename IndexType, typename VisitorType>
void visit_component(IndexType index, entt::entity entity,
                     const std::tuple<entt::basic_view<entt::entity, entt::get_t<Ts>, entt::exclude_t<>>...> &views_tuple,
                     VisitorType visitor) {
    constexpr auto table = detail::visitor_table<VisitorType, Ts...>{};
    table.array.functions[index](entity, views_tuple, visitor);
}

/*! @copydoc visit_component */
template<typename... Ts, typename IndexType, typename VisitorType>
void visit_component(std::tuple<Ts...>, IndexType index, entt::entity entity,
                     const std::tuple<entt::basic_view<entt::entity, entt::get_t<Ts>, entt::exclude_t<>>...> &views_tuple,
                     VisitorType visitor) {
    visit_component<Ts...>(index, entity, views_tuple, visitor);
}

namespace detail {
    // Similar to the ones above except this operates on a registry directly.

    // Creates a function that invokes a visitor with a component argument.
    template<typename VisitorType, typename T>
    constexpr auto make_visit_registry_function() {
        return [] (entt::entity entity, entt::registry &registry, VisitorType visitor) {
            visitor(registry.get<T>(entity));
        };
    }

    // Stores an array of visiting functions for each given type.
    template<typename VisitorType, typename... Ts>
    struct visit_registry_function_array {
        using VisitFuncType = void(*)(entt::entity, entt::registry &, VisitorType);
        std::array<VisitFuncType, sizeof...(Ts)> functions;

        constexpr visit_registry_function_array() : functions{} {
            size_t i = 0;
            ((functions[i++] = make_visit_registry_function<VisitorType, Ts>()), ...);
        }
    };

    // Holds a table of functions to visit each given component type.
    template<typename VisitorType, typename... Ts>
    struct registry_visitor_table {
        static constexpr auto array = visit_registry_function_array<VisitorType, Ts...>();
    };
}

/**
 * @brief Invokes visitor with the component of type at the index in the list
 * of provided component types in constant time.
 * @tparam Ts Family of related component types.
 * @tparam IndexType Type of integral index.
 * @tparam VisitorType Type of visitor function which takes an auto rvalue.
 * @param index Index in the family of component types of which component should
 * be visited among all types.
 * @param entity Entity of interest.
 * @param registry The source of components.
 * @param visitor Function that will be called with the desired component.
 */
template<typename... Ts, typename IndexType, typename VisitorType>
void visit_component(IndexType index, entt::entity entity,
                     entt::registry &registry, VisitorType visitor) {
    constexpr auto table = detail::registry_visitor_table<VisitorType, Ts...>{};
    table.array.functions[index](entity, registry, visitor);
}

/*! @copydoc visit_component */
template<typename... Ts, typename IndexType, typename VisitorType>
void visit_component(std::tuple<Ts...>, IndexType index, entt::entity entity,
                     entt::registry &registry, VisitorType visitor) {
    visit_component<Ts...>(index, entity, registry, visitor);
}

}

#endif // EDYN_UTIL_VISIT_COMPONENT_HPP
