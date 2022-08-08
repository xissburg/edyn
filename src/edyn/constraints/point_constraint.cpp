#include "edyn/constraints/point_constraint.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraint<point_constraint>(const entt::registry &, entt::entity, point_constraint &con,
                                          row_cache_sparse::entry &cache_entry, scalar dt,
                                          const vector3 &originA, const vector3
                                          &posA, const quaternion &ornA,
                                          const vector3 &linvelA, const vector3 &angvelA,
                                          scalar inv_mA, const matrix3x3 &inv_IA,
                                          delta_linvel &dvA, delta_angvel &dwA,
                                          const vector3 &originB,
                                          const vector3 &posB, const quaternion &ornB,
                                          const vector3 &linvelB, const vector3 &angvelB,
                                          scalar inv_mB, const matrix3x3 &inv_IB,
                                          delta_linvel &dvB, delta_angvel &dwB) {

    auto pivotA = to_world_space(con.pivot[0], originA, ornA);
    auto pivotB = to_world_space(con.pivot[1], originB, ornB);
    auto rA = pivotA - posA;
    auto rB = pivotB - posB;

    auto rA_skew = skew_matrix(rA);
    auto rB_skew = skew_matrix(rB);
    constexpr auto I = matrix3x3_identity;
    auto num_rows = size_t{3};

    for (size_t i = 0; i < num_rows; ++i) {
        auto &row = cache_entry.add_row();
        row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;

        row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = con.impulse[i];

        auto options = constraint_row_options{};
        options.error = (pivotA[i] - pivotB[i]) / dt;

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);
    }

    if (con.friction_torque > 0) {
        auto spin_axis = angvelA - angvelB;

        if (try_normalize(spin_axis)) {
            auto &row = cache_entry.add_row();
            row.J = {vector3_zero, spin_axis, vector3_zero, -spin_axis};
            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[++num_rows];

            auto friction_impulse = con.friction_torque * dt;
            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }
    }
}

}
