#ifndef EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row_data;
struct constraint;
class row_cache;

struct soft_distance_constraint {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    scalar m_relspd;

    void prepare(entt::entity, const constraint &, entt::registry &, row_cache &cache, scalar dt);
    void iteration(entt::entity, const constraint &, entt::registry &, row_cache &, size_t row_index, scalar dt);

private:
    constraint_row_data *m_damping_data;
};

}

#endif // EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP