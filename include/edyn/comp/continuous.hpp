#ifndef EDYN_COMP_CONTINUOUS_HPP
#define EDYN_COMP_CONTINUOUS_HPP

#include <unordered_set>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * Specifies a set of component types that the island worker must send back to
 * the coordinator after every step of the simulation.
 */
struct continuous {
    std::unordered_set<entt::id_type> types;

    template<typename... Component>
    void insert() {
        (types.insert(entt::type_index<Component>::value()), ...);
    }
};

}

#endif // EDYN_COMP_CONTINUOUS_HPP