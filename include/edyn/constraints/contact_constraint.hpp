#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct constraint_row_data;

struct contact_constraint : public constraint_base<contact_constraint> {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    void init(entt::entity, constraint &, entt::registry &);
    void prepare(entt::entity, constraint &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, entt::registry &, scalar dt);

private:
    scalar m_friction;
    constraint_row_data *m_normal_data;
    constraint_row_data *m_friction_data;
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP