#ifndef EDYN_UTIL_ENTT_UTIL_HPP
#define EDYN_UTIL_ENTT_UTIL_HPP

#include <tuple>
#include <vector>
#include <entt/entity/registry.hpp>

namespace edyn {

// Get a view with component types from a tuple.
template<typename... Ts>
auto get_view_from_tuple(entt::registry &registry, [[maybe_unused]] std::tuple<Ts...>) {
    return registry.view<Ts...>();
}

// Get a tuple containing a view of each given type.
template<typename... Ts>
inline auto get_tuple_of_views(entt::registry &registry) {
    return std::make_tuple(registry.view<Ts>()...);
}

// Get a tuple containing a view of each type in the given tuple.
template<typename... Ts>
inline auto get_tuple_of_views(entt::registry &registry, [[maybe_unused]] std::tuple<Ts...>) {
    return get_tuple_of_views<Ts...>(registry);
}

template<typename... Ts>
struct map_to_tuple_of_views;

// Declares a tuple type which holds single component views for each of the
// types in the given tuple.
template<typename... Ts>
struct map_to_tuple_of_views<std::tuple<Ts...>> {
    using type = std::tuple<entt::basic_view<entt::get_t<entt::registry::storage_for_type<Ts>>, entt::exclude_t<>>...>;
};

inline void entity_vector_erase_invalid(std::vector<entt::entity> &vec,
                                        const entt::registry &registry) {
    auto predicate = [&](entt::entity entity) {
        return !registry.valid(entity);
    };

    vec.erase(std::remove_if(vec.begin(), vec.end(), predicate), vec.end());
}

template<typename View>
size_t calculate_view_size(const View &view) {
    return static_cast<size_t>(std::distance(view.begin(), view.end()));
}

}

#endif // EDYN_UTIL_ENTT_UTIL_HPP
