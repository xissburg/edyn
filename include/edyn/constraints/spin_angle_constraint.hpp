#ifndef EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP

#include <vector>
#include "edyn/constraints/constraint_body.hpp"
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
    scalar m_friction_torque {1};
    bool m_engaged {true};

    struct {
        scalar spring {};
        scalar damping {};
        scalar friction {};
    } applied_impulse {};

    void set_engaged(bool, const entt::registry &);
    void set_ratio(scalar, const entt::registry &);
    scalar calculate_offset(const entt::registry &) const;
    scalar relative_velocity(const entt::registry &) const;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, spin_angle_constraint &con) {
    archive(con.body);
    archive(con.m_ratio);
    archive(con.m_stiffness);
    archive(con.m_damping);
    archive(con.m_offset_origin);
    archive(con.m_friction_torque);
    archive(con.m_engaged);
}

}

#endif // EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
