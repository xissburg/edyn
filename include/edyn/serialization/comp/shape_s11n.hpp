#ifndef EDYN_SERIALIZATION_COMP_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_COMP_SHAPE_S11N_HPP

#include "edyn/comp/shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, shape &s) {
    archive(s.var);
}

}

#endif // EDYN_SERIALIZATION_COMP_SHAPE_S11N_HPP