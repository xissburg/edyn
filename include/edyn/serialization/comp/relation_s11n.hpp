#ifndef EDYN_SERIALIZATION_COMP_RELATION_S11N_HPP
#define EDYN_SERIALIZATION_COMP_RELATION_S11N_HPP

#include "edyn/comp/relation.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, relation &rel) {
    archive(rel.entity);
}

}

#endif // EDYN_SERIALIZATION_COMP_RELATION_S11N_HPP