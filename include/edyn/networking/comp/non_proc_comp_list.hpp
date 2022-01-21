#ifndef EDYN_NETWORKING_COMP_NON_PROC_COMP_LIST_HPP
#define EDYN_NETWORKING_COMP_NON_PROC_COMP_LIST_HPP

#include "edyn/config/config.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>

namespace edyn {

/**
 * @brief A list of non-procedural components in an entity.
 * Due to the concept of ownership in networking, the server should only apply
 * state for an entity received from a client if the entity is not currently
 * owned by the server. However, exceptions must be made for certain components,
 * those which are not procedurally calculated by the physics simulation, e.g.
 * things that can be considered user input. The `non_proc_comp_list` keeps a
 * list of such components which are deemed non-procedural, and the server
 * will import the state of these components regardless of ownership status.
 * This is also used in the client to store state history which is necessary
 * during extrapolation, where the previous state of the non-procedural
 * components is applied along with the extrapolation steps, in an attempt to
 * reproduce the simulation exactly as it happened previously.
 */
struct non_proc_comp_list {
    static constexpr size_t max_size = 16;
    std::array<size_t, max_size> indices;
    size_t size {0};

    /**
     * @brief Inserts a component index to be marked as non-procedural. The
     * index must be obtained via `edyn::get_component_index<Component>()`.
     * @param index Component index.
     */
    void insert(size_t index) {
        indices[size++] = index;
        EDYN_ASSERT(size <= max_size);
    }

    /**
     * @brief Inserts multiple component indices to be marked as non-procedural.
     * The indices must be obtained via `edyn::get_component_indices<Component>()`.
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

    bool contains(size_t index) const {
        auto end = indices.begin();
        std::advance(end, size);
        return std::find(indices.begin(), end, index) != end;
    }
};

template<typename Archive>
void serialize(Archive &archive, non_proc_comp_list &c) {
    archive(c.size);
    EDYN_ASSERT(c.size < c.max_size);

    for (unsigned i = 0; i < c.size; ++i) {
        archive(c.indices[i]);
    }
}

}

#endif // EDYN_NETWORKING_COMP_NON_PROC_COMP_LIST_HPP
