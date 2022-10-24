#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"

namespace edyn {

struct constraint_row_prep_cache;
class position_solver;
struct quaternion;
struct matrix3x3;

/**
 * @brief Constrains two points on a pair of rigid bodies to match in space.
 */
struct point_constraint : public constraint_base {
    // Pivot points in object space.
    std::array<vector3, 2> pivot;

    scalar friction_torque{};

    std::array<scalar, 3> applied_impulse {};
    scalar applied_friction_impulse {};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, point_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.friction_torque);
    archive(c.applied_impulse);
    archive(c.applied_friction_impulse);
}

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
