#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct contact_constraint : public constraint_base<contact_constraint> {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP