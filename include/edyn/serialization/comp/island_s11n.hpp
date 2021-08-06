#ifndef EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP
#define EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP

#include "edyn/comp/island.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, island &isle) {
}

template<typename Archive>
void serialize(Archive &archive, island_timestamp &timestamp) {
    archive(timestamp.value);
}

}

#endif // EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP