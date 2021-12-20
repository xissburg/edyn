#ifndef EDYN_SERIALIZATION_COMP_S11N_HPP
#define EDYN_SERIALIZATION_COMP_S11N_HPP

#include "edyn/comp/continuous.hpp"
#include "edyn/config/config.h"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, continuous &c) {
    archive(c.size);
    EDYN_ASSERT(c.size < c.max_size);

    for (unsigned i = 0; i < c.size; ++i) {
        archive(c.indices[i]);
    }
}

}

#endif // EDYN_SERIALIZATION_COMP_S11N_HPP
