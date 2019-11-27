#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct point_constraint : public constraint_base<point_constraint> {
    std::array<vector3, 2> pivot;

    void init(constraint &, const relation &, entt::registry &);
    void prepare(constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP