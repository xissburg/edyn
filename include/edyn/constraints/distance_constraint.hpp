#ifndef EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP

#include <array>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct distance_constraint : public constraint_base<distance_constraint> {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP