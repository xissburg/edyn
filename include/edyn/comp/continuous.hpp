#ifndef EDYN_COMP_CONTINUOUS_HPP
#define EDYN_COMP_CONTINUOUS_HPP

#include "edyn/config/config.h"
#include <array>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * Specifies a set of component types that the island worker must send back to
 * the coordinator after every step of the simulation.
 */
struct continuous {
    static constexpr size_t max_size = 16;
    std::array<entt::id_type, max_size> types;
    size_t size {0};

    template<typename... Component>
    void insert() {
        ((types[size++] = entt::type_index<Component>::value()), ...);
        EDYN_ASSERT(size <= max_size);
    }

    template<typename Component>
    void remove() {
        for (size_t i = 0; i < size; ++i) {
            if (types[i] == entt::type_index<Component>::value()) {
                types[i] = types[--size];
                break;
            }
        }
    }
};

}

#endif // EDYN_COMP_CONTINUOUS_HPP