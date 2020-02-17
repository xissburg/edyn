#ifndef EDYN_CONSTRAINTS_TIRE_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRE_CONTACT_CONSTRAINT_HPP

#include "constraint_base.hpp"
#include "edyn/util/tire.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct tire_contact_constraint : public constraint_base<tire_contact_constraint> {
    struct tire_contact {
        std::array<entt::entity, 3> row_entity {make_array<3>(entt::entity{entt::null})};
        vector3 lon_dir;
        vector3 lat_dir;
        vector3 last_impulse;
        vector3 last_applied_impulse;
        scalar  last_lateral_impulse;
        scalar  last_normal_impulse;
        scalar  lon_slip;
        scalar  lat_slip;
        vector3 rel_vel;
        scalar  slide_factor;
        scalar  deflection;
        scalar  contact_patch_length;
        scalar  contact_patch_width;
        scalar  slip_angle;
    };

    std::array<tire_contact, max_contacts> contacts;
    tire_specification spec;

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);

    vector3 calculate_tire_impulses(
        const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &spinvelA,
        const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        const vector3 &spinvelB,
        const contact_point &cp, 
        tire_contact_constraint::tire_contact &contact, 
        scalar normal_impulse, scalar dt);
}; 

}

#endif // EDYN_CONSTRAINTS_TIRE_CONTACT_CONSTRAINT_HPP