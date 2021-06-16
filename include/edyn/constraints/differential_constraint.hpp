#ifndef EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct differential_constraint {
    std::array<entt::entity, 3> body {entt::null, entt::null, entt::null};
    scalar ratio;
};

template<>
void prepare_constraints<differential_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP