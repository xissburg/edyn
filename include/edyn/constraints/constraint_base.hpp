#ifndef EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/config/constants.hpp"

namespace edyn {

struct constraint_base {
    std::array<entt::entity, 2> body;
    std::array<scalar, max_constraint_rows> impulse;
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP
