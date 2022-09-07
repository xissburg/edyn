#ifndef EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP

#include <array>
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct vector3;
struct quaternion;

/**
 * Constraints the `spin_angle` of two entities, which can be constrained
 * to spin at different rates using a gear ratio.
 */
struct spin_angle_constraint : public constraint_base {
    scalar m_ratio {1};
    scalar m_stiffness {1e5};
    scalar m_damping {1e2};
    scalar m_offset_origin {0};

    std::array<scalar, 2> impulse {0, 0};

    void set_ratio(scalar, const entt::registry &);
    scalar calculate_offset(const entt::registry &) const;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB);
};

template<typename Archive>
void serialize(Archive &archive, spin_angle_constraint &con) {
    archive(con.body);
    archive(con.m_ratio);
    archive(con.m_stiffness);
    archive(con.m_damping);
    archive(con.m_offset_origin);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
