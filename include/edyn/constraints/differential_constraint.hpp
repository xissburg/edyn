#ifndef EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct vector3;
struct quaternion;
struct constraint_row_prep_cache;

struct differential_constraint {
    std::array<entt::entity, 3> body {entt::null, entt::null, entt::null};
    scalar ratio;
    scalar applied_impulse;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, differential_constraint &con) {
    archive(con.body);
    archive(con.ratio);
    archive(con.applied_impulse);
}

}

#endif // EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
