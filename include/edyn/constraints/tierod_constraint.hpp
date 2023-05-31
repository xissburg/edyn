#ifndef EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP

#include <vector>
#include <entt/fwd.hpp>
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct quaternion;

struct tierod_offset {
    vector3 value {vector3_zero};
};

struct tierod_constraint : public constraint_base {
    vector3 pivotA;
    vector3 pivotB;
    scalar rod_length;

    // upper control arm pivots
    vector3 upper_pivotA;
    vector3 upper_pivotB;

    // lower control arm pivots
    vector3 lower_pivotA;
    vector3 lower_pivotB;

    // length of control arms
    scalar upper_length;
    scalar lower_length;

    // Steering axis in wheel object space.
    vector3 steering_axis;

    // Vector from steering axis to pivot1
    vector3 steering_arm;
    scalar steering_arm_length;
    scalar steering_arm_angle;

    scalar applied_impulse;

    void update_steering_axis();
    void update_steering_arm();

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, tierod_constraint &con) {
    archive(con.body);
    archive(con.pivotA);
    archive(con.pivotB);
    archive(con.rod_length);
    archive(con.upper_pivotA);
    archive(con.upper_pivotB);
    archive(con.lower_pivotA);
    archive(con.lower_pivotB);
    archive(con.upper_length);
    archive(con.lower_length);
    archive(con.steering_axis);
    archive(con.steering_arm);
    archive(con.steering_arm_length);
    archive(con.steering_arm_angle);
    archive(con.applied_impulse);
}

template<typename Archive>
void serialize(Archive &archive, tierod_offset &offset) {
    archive(offset.value);
}

}

#endif // EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP
