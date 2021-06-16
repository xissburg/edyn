#ifndef EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP

#include <array>
#include <entt/entity/fwd.hpp>

namespace edyn {

struct constraint_base {
    std::array<entt::entity, 2> body;
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP
