#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct contact_constraint : public constraint_base {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    scalar m_friction;
};

template<>
void prepare_constraints<contact_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
void iterate_constraints<contact_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
