#ifndef EDYN_COMP_CONTINUOUS_HPP
#define EDYN_COMP_CONTINUOUS_HPP

#include "edyn/config/config.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace edyn {

/**
 * @brief Specifies a set of component types that the island worker must send
 * back to the coordinator after every step of the simulation.
 * @remark The types are stored as the index of the component in the current
 * `component_index_source` as to make them stable among different machines to
 * allow this component to be shared between client and server in a networked
 * simulation.
 */
struct continuous {
    using index_type = uint16_t;
    using size_type = uint8_t;
    static constexpr size_type max_size = 16;
    std::array<index_type, max_size> indices;
    size_type size {0};

    /**
     * @brief Inserts a component index to be marked as continuous. The index
     * must be obtained via `edyn::get_component_index<Component>()`.
     * @param index Component index.
     */
    void insert(index_type index) {
        EDYN_ASSERT(size < max_size);
        indices[size++] = index;
    }

    /**
     * @brief Inserts multiple component indices to be marked as continuous. The
     * indices must be obtained via `edyn::get_component_indices<Component>()`.
     * @param indices Component indices.
     */
    template<size_t N>
    void insert(const std::array<index_type, N> &indices) {
        for (auto i : indices) {
            insert(i);
        }
    }

    /**
     * @brief Removes a component with the given index. The index must be
     * obtained via `edyn::get_component_index<Component>()`.
     * @param index Component index.
     */
    void remove(index_type index) {
        for (size_type i = 0; i < size; ++i) {
            if (indices[i] == index) {
                indices[i] = indices[--size];
                break;
            }
        }
    }
};

template<typename Archive>
void serialize(Archive &archive, continuous &c) {
    archive(c.size);
    EDYN_ASSERT(c.size < c.max_size);

    for (unsigned i = 0; i < c.size; ++i) {
        archive(c.indices[i]);
    }
}

}

#endif // EDYN_COMP_CONTINUOUS_HPP
