#ifndef EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct constraint_body;

struct soft_distance_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    scalar applied_spring_impulse {};
    scalar applied_damping_impulse {};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, soft_distance_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.distance);
    archive(c.stiffness);
    archive(c.damping);
    archive(c.applied_spring_impulse);
    archive(c.applied_damping_impulse);
}

}

#endif // EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
