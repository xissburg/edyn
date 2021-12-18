#ifndef EDYN_COMP_CONTINUOUS_HPP
#define EDYN_COMP_CONTINUOUS_HPP

#include "edyn/config/config.h"
#include <array>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * @brief Specifies a set of component types that the island worker must send
 * back to the coordinator after every step of the simulation.
 * @remark The types are stored as the index of the component in the current
 * `component_source_index` as to make them stable among different machines to
 * allow this component to be shared between client and server in a networked
 * simulation.
 */
struct continuous {
    static constexpr size_t max_size = 16;
    std::array<size_t, max_size> indices;
    size_t size {0};

    /**
     * @brief Inserts a component index to be marked as continuous. The index
     * must be obtained via `edyn::get_component_index<Component>()`.
     * @param index Component index.
     */
    void insert(size_t index) {
        indices[size++] = index;
        EDYN_ASSERT(size <= max_size);
    }

    /**
     * @brief Inserts multiple component indices to be marked as continuous. The
     * indices must be obtained via `edyn::get_component_indices<Component>()`.
     * @param indices Component indices.
     */
    template<size_t N>
    void insert(const std::array<size_t, N> &indices) {
        for (auto i : indices) {
            insert(i);
        }
    }

    /**
     * @brief Removes a component with the given index. The index must be
     * obtained via `edyn::get_component_index<Component>()`.
     * @param index Component index.
     */
    void remove(size_t index) {
        for (size_t i = 0; i < size; ++i) {
            if (indices[i] == index) {
                indices[i] = indices[--size];
                break;
            }
        }
    }
};

}

#endif // EDYN_COMP_CONTINUOUS_HPP
