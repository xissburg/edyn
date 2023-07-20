#ifndef EDYN_CONSTRAINTS_TRIPLE_SPIN_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TRIPLE_SPIN_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row_prep_cache;

struct triple_spin_constraint {
    std::array<entt::entity, 3> body {entt::null, entt::null, entt::null};
    std::array<scalar, 3> m_ratio {scalar(1), scalar(1), scalar(1)};
    std::array<vector3, 3> m_axis {vector3_x, vector3_x, vector3_x};
    std::array<bool, 3> m_use_spin {true, true, true};

    scalar applied_impulse {};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, triple_spin_constraint &con) {
    archive(con.body);
    archive(con.m_ratio);
    archive(con.m_axis);
    archive(con.m_use_spin);
    archive(con.applied_impulse);
}

}

#endif // EDYN_CONSTRAINTS_TRIPLE_SPIN_CONSTRAINT_HPP
