#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include "constraint_base.hpp"

namespace edyn {

struct tirecarcass_constraint : public constraint_base<tirecarcass_constraint> {
    scalar m_lateral_stiffness {120000};
    scalar m_lateral_damping {40};
    scalar m_longitudinal_stiffness {2000};
    scalar m_longitudinal_damping {30};
    scalar m_torsional_stiffness {1000};
    scalar m_torsional_damping {20};

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP