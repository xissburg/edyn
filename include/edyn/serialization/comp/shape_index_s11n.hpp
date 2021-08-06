#ifndef EDYN_SERIALIZATION_COMP_SHAPE_INDEX_S11N_HPP
#define EDYN_SERIALIZATION_COMP_SHAPE_INDEX_S11N_HPP

#include "edyn/comp/shape_index.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, shape_index &index) {
    archive(index.value);
}

}

#endif // EDYN_SERIALIZATION_COMP_SHAPE_INDEX_S11N_HPP
