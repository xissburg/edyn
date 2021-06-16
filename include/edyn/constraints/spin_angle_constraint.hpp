#ifndef EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

/**
 * Constraints the `spin_angle` of two entities, which can be constrained
 * to spin at different rates using a gear ratio.
 */
struct spin_angle_constraint : public constraint_base {
    scalar m_ratio {1};
    scalar m_stiffness {1e5};
    scalar m_damping {1e2};
    scalar m_offset_origin {0};

    void set_ratio(scalar, const entt::registry &);
    scalar calculate_offset(const entt::registry &) const;
};

template<>
void prepare_constraints<spin_angle_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP