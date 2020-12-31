#ifndef EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP

#include "constraint_base.hpp"

namespace edyn {

/**
 * Constrains the `spin` of two entities to keep them spinning at the same
 * speed. The `m_max_impulse` can be set to a lower value to allow them to
 * _slide_ if the torque gets above a threshold.
 */
struct spin_constraint : public constraint_base<spin_constraint> {
    scalar m_max_torque;

    void init(entt::entity, constraint &, entt::registry &);
    void prepare(entt::entity, constraint &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP