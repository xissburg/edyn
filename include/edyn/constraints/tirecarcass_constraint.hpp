#ifndef EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP

#include <array>
#include <vector>
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct constraint_row_prep_cache;

struct tirecarcass_constraint : public constraint_base {
    scalar m_lateral_stiffness {120000};
    scalar m_lateral_damping {40};
    scalar m_longitudinal_stiffness {2000};
    scalar m_longitudinal_damping {30};

    struct {
        scalar lateral_spring {};
        scalar lateral_damping {};
        scalar vertical {};
        scalar longitudinal {};
        std::array<scalar, 3> rotational {};
        scalar longitudinal_twist_spring {};
        scalar longitudinal_twist_damping {};
    } applied_impulse;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, tirecarcass_constraint &con) {
    archive(con.body);
    archive(con.m_lateral_stiffness);
    archive(con.m_lateral_damping);
    archive(con.m_longitudinal_stiffness);
    archive(con.m_longitudinal_damping);
}

}

#endif // EDYN_CONSTRAINTS_TIRECARCASS_CONSTRAINT_HPP
