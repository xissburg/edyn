#ifndef EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP

#include "edyn/collision/contact_point.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, contact_point &cp) {
    archive(cp.pivotA);
    archive(cp.pivotB);
    archive(cp.normal);
    archive(cp.local_normal);
    archive(cp.normal_attachment);
    archive(cp.friction);
    archive(cp.spin_friction);
    archive(cp.roll_friction);
    archive(cp.restitution);
    archive(cp.stiffness);
    archive(cp.damping);
    archive(cp.lifetime);
    archive(cp.distance);
    archive(cp.featureA);
    archive(cp.featureB);
    archive(cp.normal_impulse);
    archive(cp.friction_impulse);
    archive(cp.spin_friction_impulse);
    archive(cp.rolling_friction_impulse);
    archive(cp.normal_restitution_impulse);
    archive(cp.friction_restitution_impulse);
}

}

#endif // EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP
