#ifndef EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP

#include <array>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct generic_constraint : public constraint_base<generic_constraint> {
    std::array<vector3, 2> pivot;

    void init(entt::entity, constraint &, entt::registry &);
    void prepare(entt::entity, constraint &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP