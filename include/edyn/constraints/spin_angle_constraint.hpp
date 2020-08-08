#ifndef EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP

#include "constraint_base.hpp"

namespace edyn {

/**
 * Constraints the `spin_angle` of two entities, which can be constrained
 * to spin at different rates using a gear ratio.
 */
struct spin_angle_constraint : public constraint_base<spin_angle_constraint> {
    scalar m_ratio {1};
    scalar m_stiffness {1e5};
    scalar m_damping {1e2};
    scalar m_offset_origin {0};

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);

    void set_ratio(scalar, const relation &, entt::registry &);
    scalar calculate_offset(const relation &, entt::registry &) const;
};

}

#endif // EDYN_CONSTRAINTS_SPIN_ANGLE_CONSTRAINT_HPP