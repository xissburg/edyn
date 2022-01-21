#include "edyn/constraints/cone_constraint.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
#include <cmath>

namespace edyn {

template<>
void prepare_constraints<cone_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<cone_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (cone_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        // Transform B's pivot into the frame space in A and apply scaling so
        // that the cone is circular and has an opening angle of 90 degrees, which
        // makes calculations easier. The transformation is later reverted to
        // gather results in world space.
        auto pivotB_world = to_world_space(con.pivot[1], originB, ornB);
        auto pivotB_in_A = to_object_space(pivotB_world, originA, ornA);
        auto pivotB_in_A_frame = to_object_space(pivotB_in_A, con.pivot[0], con.frame);

        // Scaling to make the cone circular with an opening of 90 degrees.
        auto scaling_y = scalar(1) / con.span_tan[0];
        auto scaling_z = scalar(1) / con.span_tan[1];
        auto pivotB_in_A_frame_scaled = pivotB_in_A_frame * vector3{1, scaling_y, scaling_z};

        // Calculate normal vector on cone which points towards the pivot.
        auto proj_yz_len_sqr = length_sqr(to_vector2_yz(pivotB_in_A_frame_scaled));
        vector3 normal_scaled, tangent_scaled;

        if (proj_yz_len_sqr > EDYN_EPSILON) {
            normal_scaled = normalize(vector3{-std::sqrt(proj_yz_len_sqr),
                                              pivotB_in_A_frame_scaled.y,
                                              pivotB_in_A_frame_scaled.z});
            tangent_scaled = normalize(vector3{0, -pivotB_in_A_frame_scaled.z, pivotB_in_A_frame_scaled.y});
        } else {
            normal_scaled = normalize(vector3{-1, 1, 0});
            tangent_scaled = normalize(vector3{0, 0, 1});
        }

        auto error = dot(pivotB_in_A_frame_scaled, normal_scaled);

        // Find point cone closest to B's pivot.
        auto dir_on_cone = vector3{-normal_scaled.x, normal_scaled.y, normal_scaled.z};
        auto cone_proj = dot(pivotB_in_A_frame_scaled, dir_on_cone);
        auto point_on_cone_scaled = dir_on_cone * cone_proj;
        auto point_on_cone = point_on_cone_scaled * vector3{1, 1 / scaling_y, 1 / scaling_z};

        auto pivotA = to_world_space(point_on_cone, con.pivot[0], con.frame);
        auto pivotA_world = to_world_space(pivotA, originA, ornA);

        // The tangent to a circle continues to be a tangent of the ellipse after
        // both are scaled, unlike the normal vector. Thus, unscale the tangent
        // and recalculate the normal.
        auto tangent = normalize(tangent_scaled * vector3{1, 1 / scaling_y, 1 / scaling_z});
        auto normal = normalize(cross(tangent, point_on_cone));
        auto normal_world = rotate(ornA, con.frame * normal);

        auto rA = pivotA_world - posA;
        auto rB = pivotB_world - posB;
        unsigned row_idx = 0;

        std::array<vector3, 2 * max_constrained_entities> J =
            {normal_world,  cross(rA, normal_world),
            -normal_world, -cross(rB, normal_world)};

        auto &row = cache.rows.emplace_back();
        row.J = J;
        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.lower_limit = 0;
        row.upper_limit = large_scalar;
        row.impulse = con.impulse[row_idx++];

        auto options = constraint_row_options{};
        options.error = -error / dt;
        options.restitution = con.restitution;

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);

        if (con.bump_stop_stiffness > 0 && con.bump_stop_length > 0) {
            auto &row = cache.rows.emplace_back();
            row.J = J;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto bump_stop_deflection = con.bump_stop_length + error;
            auto spring_force = con.bump_stop_stiffness * bump_stop_deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = 0;
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = -bump_stop_deflection / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

}
