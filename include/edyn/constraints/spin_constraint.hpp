#ifndef EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct vector3;
struct quaternion;
struct constraint_row_prep_cache;

/**
 * Constrains the `spin` of two entities to keep them spinning at the same
 * speed. The `m_max_torque` can be set to a lower value to allow them to
 * _slide_ if the torque gets above a threshold.
 */
struct spin_constraint : public constraint_base {
    scalar m_max_torque {};
    bool m_use_spinA {true};
    bool m_use_spinB {true};

    scalar impulse {};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB);
};

template<typename Archive>
void serialize(Archive &archive, spin_constraint &con) {
    archive(con.body);
    archive(con.m_max_torque);
    archive(con.m_use_spinA);
    archive(con.m_use_spinB);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP
