#ifndef EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP
#define EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP

#include "edyn/comp/island.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, island &isle) {
    archive(isle.entities);
    archive(isle.sleep_step);
}

template<typename Archive>
void serialize(Archive &archive, island_node &node) {
    archive(node.entities);
}

}

#endif // EDYN_SERIALIZATION_COMP_ISLAND_S11N_HPP