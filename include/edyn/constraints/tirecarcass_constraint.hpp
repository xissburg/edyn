#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct tirecarcass_constraint : public constraint_base {
    scalar m_lateral_stiffness {120000};
    scalar m_lateral_damping {40};
    scalar m_longitudinal_stiffness {2000};
    scalar m_longitudinal_damping {30};
    scalar m_torsional_stiffness {1000};
    scalar m_torsional_damping {20};

    scalar m_lateral_relspd;
    scalar m_longitudinal_relspd;
    scalar m_torsional_relspd;
};

template<>
void prepare_constraints<tirecarcass_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
void iterate_constraints<tirecarcass_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP