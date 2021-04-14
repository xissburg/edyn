#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

static constexpr size_t max_constrained_entities = 2;

/**
 * @brief A constraint row represents a restriction of one degree of freedom
 * between two rigid bodies. 
 * 
 * It is split in two separate components so that
 * during the constraint solver iterations only the necessary data is used,
 * for performance reasons.
 */
struct constraint_row {
    std::array<entt::entity, max_constrained_entities> entity;

    scalar error;

    // Error reduction parameter.
    scalar erp {0.2};

    scalar restitution {0};

    int priority {0};
};

struct delta_linvel;
struct delta_angvel;

/**
 * `constraint_row_data` contains all and only the information that's required
 * during the constraint solver iterations for better cache use and to avoid
 * waste.
 */
struct constraint_row_data {
    // Jacobian diagonals.
    std::array<vector3, 2 * max_constrained_entities> J;

    // Effective mass (J M^-1 J^T)^-1.
    scalar eff_mass;

    // Right hand side Jv + bias.
    scalar rhs;

    // Lower and upper limit of impulses to be applied while solving constraints.
    scalar lower_limit;
    scalar upper_limit;

    // Last applied impulse. Used for warm-starting and can be used to measure
    // strength of impulse applied.
    scalar impulse;

    // Inverse masses and inertias used during the solver iterations. Values
    // do not necessarily represent the latest state. Query values for the
    // rigid body from the registry instead.
    scalar inv_mA, inv_mB;
    matrix3x3 inv_IA, inv_IB;

    // Reference to delta velocities used during solver iterations. It is not
    // safe to dereference these outside of the solver context.
    delta_linvel *dvA, *dvB;
    delta_angvel *dwA, *dwB;
};

struct constraint_row_impulse {
    scalar value;
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP