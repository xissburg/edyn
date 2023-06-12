#ifndef EDYN_DYNAMICS_POSITION_SOLVER_HPP
#define EDYN_DYNAMICS_POSITION_SOLVER_HPP

#include "edyn/comp/inertia.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

class position_solver {
public:

    void solve(const std::array<vector3, 4> &J, scalar error) {
        auto eff_mass = get_effective_mass(J, inv_mA, *inv_IA, inv_mB, *inv_IB);
        auto correction = error * error_correction_rate * eff_mass;

        // Apply position correction.
        *posA += inv_mA * J[0] * correction;
        *posB += inv_mB * J[2] * correction;

        // Use quaternion derivative to apply angular correction which should
        // be good enough for small angles.
        auto angular_correctionA = *inv_IA * J[1] * correction;
        *ornA += quaternion_derivative(*ornA, angular_correctionA);
        *ornA = normalize(*ornA);

        auto angular_correctionB = *inv_IB * J[3] * correction;
        *ornB += quaternion_derivative(*ornB, angular_correctionB);
        *ornB = normalize(*ornB);

        // Compute origins with new transforms.
        if (originA) {
            *originA = to_world_space(-comA, *posA, *ornA);
        }

        if (originB) {
            *originB = to_world_space(-comB, *posB, *ornB);
        }

        // Compute world space inertia with new orientation.
        auto basisA = to_matrix3x3(*ornA);
        *inv_IA = basisA * *inv_IA_local * transpose(basisA);

        auto basisB = to_matrix3x3(*ornB);
        *inv_IB = basisB * *inv_IB_local * transpose(basisB);

        max_error = std::max(std::abs(error), max_error);
    }

    vector3 get_originA() {
        return originA ? static_cast<vector3>(*originA) : static_cast<vector3>(*posA);
    }

    vector3 get_originB() {
        return originB ? static_cast<vector3>(*originB) : static_cast<vector3>(*posB);
    }

    origin *originA, *originB;
    vector3 comA, comB;
    position *posA, *posB;
    orientation *ornA, *ornB;
    scalar inv_mA, inv_mB;
    inertia_world_inv *inv_IA, *inv_IB;
    inertia_inv *inv_IA_local, *inv_IB_local;
    scalar error_correction_rate {0.2};
    scalar max_error {};
};

}

#endif // EDYN_DYNAMICS_POSITION_SOLVER_HPP
