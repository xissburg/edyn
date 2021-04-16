#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct row_cache;

struct contact_constraint : public constraint_base {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    scalar m_friction;
};

void prepare_contact_constraints(entt::registry &, row_cache &, scalar dt);
void iterate_contact_constraints(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
