#ifndef EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPIN_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

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
};

template<>
void prepare_constraints<spin_constraint>(entt::registry &, row_cache &, scalar dt);

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
