#ifndef EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/util/array_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct constraint_row_prep_cache;
class position_solver;
struct quaternion;
struct matrix3x3;

struct soft_distance_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    std::array<scalar, 2> impulse {make_array<2, scalar>(0)};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        scalar inv_mA, const matrix3x3 &inv_IA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        scalar inv_mB, const matrix3x3 &inv_IB);
};

template<typename Archive>
void serialize(Archive &archive, soft_distance_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.distance);
    archive(c.stiffness);
    archive(c.damping);
    archive(c.impulse);
}

}

#endif // EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
