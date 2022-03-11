#ifndef EDYN_SERIALIZATION_COMP_COLLISION_EXCLUSION_S11N_HPP
#define EDYN_SERIALIZATION_COMP_COLLISION_EXCLUSION_S11N_HPP

#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/config/config.h"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, collision_exclusion &excl) {
    auto num_entities = excl.num_entities();
    archive(num_entities);
    EDYN_ASSERT(num_entities < excl.max_exclusions);

    for (unsigned i = 0; i < num_entities; ++i) {
        archive(excl.entity[i]);
    }
}

}

#endif // EDYN_SERIALIZATION_COMP_COLLISION_EXCLUSION_S11N_HPP
