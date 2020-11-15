#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

static constexpr size_t max_constrained_entities = 2;

struct constraint_row {
    std::array<entt::entity, max_constrained_entities> entity =
        make_array<max_constrained_entities>(entt::entity{entt::null});

    std::array<vector3, max_constrained_entities * 2> J;
    scalar error;
    scalar lower_limit;
    scalar upper_limit;

    // Effective mass (J M^-1 J^T)^-1.
    scalar eff_mass;

    // Right hand side Jv + bias.
    scalar rhs;

    // Error reduction parameter.
    scalar erp {0.2};

    // Relative velocity at pivot points.
    scalar relvel;
    scalar restitution {0};

    // Applied impulse.
    scalar impulse {0};
    int priority {0};
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP