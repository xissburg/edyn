#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct quaternion;
struct vector3;

struct tirecarcass_constraint : public constraint_base {
    scalar m_lateral_stiffness {120000};
    scalar m_lateral_damping {40};
    scalar m_longitudinal_stiffness {2000};
    scalar m_longitudinal_damping {30};

    static const auto num_rows = 9;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB);
};

template<typename Archive>
void serialize(Archive &archive, tirecarcass_constraint &con) {
    archive(con.body);
    archive(con.m_lateral_stiffness);
    archive(con.m_lateral_damping);
    archive(con.m_longitudinal_stiffness);
    archive(con.m_longitudinal_damping);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
