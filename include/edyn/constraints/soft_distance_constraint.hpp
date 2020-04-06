#ifndef EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP

#include <array>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct soft_distance_constraint : public constraint_base<soft_distance_constraint> {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    scalar m_relspd;

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);

};

}

#endif // EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP