#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include "edyn/collision/contact_normal_attachment.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include <vector>

namespace edyn {

class position_solver;
struct constraint_row_prep_cache;

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {
    vector3 pivotA;
    vector3 pivotB;
    vector3 normal;

    vector3 local_normal; // Normal in object space.
    contact_normal_attachment normal_attachment;
    scalar distance; // Signed distance along normal.

    scalar friction;
    scalar restitution;

    scalar applied_normal_impulse {scalar(0)};
    std::array<scalar, 2> applied_friction_impulse {scalar(0), scalar(0)};

    void prepare(constraint_row_prep_cache &cache, scalar dt,
                 const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &solver);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
    // The remaining members hold calculated values.
}

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
