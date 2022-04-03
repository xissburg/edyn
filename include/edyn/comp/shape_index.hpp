#ifndef EDYN_COMP_SHAPE_INDEX_HPP
#define EDYN_COMP_SHAPE_INDEX_HPP

#include <cstdint>

namespace edyn {

/**
 * @brief Stores the index of the type of shape held by the entity which has
 * this component. The actual shape can be obtained using `visit_shape`. Shape
 * indices must be generated using `get_shape_index`.
 */
struct shape_index {
    using index_type = uint8_t;
    index_type value;
};

template<typename Archive>
void serialize(Archive &archive, shape_index &index) {
    archive(index.value);
}

}

#endif // EDYN_COMP_SHAPE_INDEX_HPP
