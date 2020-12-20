#ifndef EDYN_SERIALIZATION_COMP_LINACC_S11N_HPP
#define EDYN_SERIALIZATION_COMP_LINACC_S11N_HPP

#include "edyn/comp/linacc.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, linacc &a) {
    archive(a.x, a.y, a.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_LINACC_S11N_HPP