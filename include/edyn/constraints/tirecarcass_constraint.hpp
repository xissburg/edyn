#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include "constraint_base.hpp"

namespace edyn {

struct tirecarcass_constraint : public constraint_base<tirecarcass_constraint> {
    scalar m_lateral_stiffness;
    scalar m_lateral_damping;
    scalar m_longitudinal_stiffness;
    scalar m_longitudinal_damping;
    scalar m_torsional_stiffness;
    scalar m_torsional_damping;

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP