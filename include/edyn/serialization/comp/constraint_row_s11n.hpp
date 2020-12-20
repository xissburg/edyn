#ifndef EDYN_SERIALIZATION_COMP_CONSTRAINT_ROW_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONSTRAINT_ROW_S11N_HPP

#include "edyn/comp/constraint_row.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, constraint_row &r) {
    archive(r.entity);
    archive(r.J);
    archive(r.error);
    archive(r.lower_limit);
    archive(r.upper_limit);
    archive(r.eff_mass);
    archive(r.rhs);
    archive(r.erp);
    archive(r.relvel);
    archive(r.restitution);
    archive(r.impulse);
    archive(r.priority);
}

}

#endif // EDYN_SERIALIZATION_COMP_CONSTRAINT_ROW_S11N_HPP