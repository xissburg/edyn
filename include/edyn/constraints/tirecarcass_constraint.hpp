#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct tirecarcass_constraint : public constraint_base {
    scalar m_lateral_stiffness {120000};
    scalar m_lateral_damping {40};
    scalar m_longitudinal_stiffness {2000};
    scalar m_longitudinal_damping {30};

    static const auto num_rows = 7;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});
};

template<>
void prepare_constraints<tirecarcass_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
