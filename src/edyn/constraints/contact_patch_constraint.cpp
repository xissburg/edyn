#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/config/config.h"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/tire_util.hpp"
#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

std::pair<vector3, vector3> get_tire_directions(vector3 axis, vector3 normal, quaternion orn) {
    auto lat_dir = project_direction(axis, normal);
    auto lon_dir = vector3_zero;

    if (try_normalize(lat_dir)) {
        lon_dir = cross(lat_dir, normal);
    } else {
        auto tire_z = quaternion_z(orn);
        lon_dir = tire_z - normal * dot(tire_z, normal);
        lon_dir = normalize(lon_dir);
        lat_dir = cross(normal, lon_dir);
    }

    return {lon_dir, lat_dir};
}

bool intervals_intersect(scalar min_a, scalar max_a, scalar min_b, scalar max_b) {
    EDYN_ASSERT(min_a <= max_a);
    EDYN_ASSERT(min_b <= max_b);
    return min_a <= max_b && max_a >= min_b;
}

bool intervals_intersect_wrap_around(scalar min_a, scalar max_a, scalar min_b, scalar max_b, scalar range_min, scalar range_max) {
    EDYN_ASSERT(min_a >= range_min && min_a <= range_max);
    EDYN_ASSERT(min_b >= range_min && min_b <= range_max);
    EDYN_ASSERT(max_a >= range_min && max_a <= range_max);
    EDYN_ASSERT(max_b >= range_min && max_b <= range_max);

    if (min_a <= max_a && min_b <= max_b) {
        return intervals_intersect(min_a, max_a, min_b, max_b);
    }

    if (min_a > max_a) {
        if (min_b > max_b) {
            // If both wrap around then they intersect as a consequence.
            return true;
        } else {
            return intervals_intersect(range_min, max_a, min_b, max_b) ||
                   intervals_intersect(min_a, range_max, min_b, max_b);
        }
    }

    EDYN_ASSERT(min_b > max_b);
    return intervals_intersect(min_a, max_a, range_min, max_b) ||
           intervals_intersect(min_a, max_a, min_b, range_max);
}

void contact_patch_constraint::prepare(const entt::registry &registry, entt::entity entity, const contact_manifold &manifold,
                                       constraint_row_prep_cache &cache, scalar dt,
                                       const constraint_body &bodyA, const constraint_body &bodyB) {
    if (manifold.num_points == 0) {
        num_patches = 0;
        return;
    }

    // Wheel spin axis in world space.
    const auto axis = quaternion_x(bodyA.orn);
    auto spin_axisA = axis;
    auto spin_axisB = quaternion_x(bodyB.orn);

    auto spin_ornA = bodyA.orn * quaternion_axis_angle(vector3_x, bodyA.spin_angle);

    auto &cyl = registry.get<cylinder_shape>(body[0]);

    // Create non-penetration constraint rows for each contact point.
    // Ignore spin for normal constraint since it only affects tangential
    // directions and for cylinders the normal always points towards the
    // spin axis.
    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);

        EDYN_ASSERT(length_sqr(cp.normal) > EDYN_EPSILON);
        auto normal = cp.normal;
        auto pivotA = to_world_space(cp.pivotA, bodyA.origin, bodyA.orn);
        auto pivotB = to_world_space(cp.pivotB, bodyB.origin, bodyB.orn);
        auto rA = pivotA - bodyA.pos;
        auto rB = pivotB - bodyB.pos;

        auto &row = cache.add_row_with_spin();
        row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        row.impulse = cp.normal_impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = spin_axisA;
        row.spin_axis[1] = spin_axisB;
        row.lower_limit = 0;

        auto deflection = std::max(-cp.distance, scalar(0));
        auto local_travel_speed = bodyA.spin * (cyl.radius - deflection);
        auto stiffness = velocity_dependent_vertical_stiffness(cp.stiffness,
                                                               std::abs(local_travel_speed));

        // Divide stiffness by number of points in the same contact plane
        // for correct force distribution.
        unsigned num_points_in_same_plane = 0;

        for (unsigned i = 0; i < manifold.num_points; ++i) {
            auto &other_cp = manifold.get_point(i);
            auto other_pivotB = to_world_space(other_cp.pivotB, bodyB.origin, bodyB.orn);

            if (std::abs(dot(pivotB - other_pivotB, normal)) < collision_threshold) {
                ++num_points_in_same_plane;
            }
        }

        auto spring_force = deflection * stiffness / num_points_in_same_plane;

        auto vA = bodyA.linvel + cross(bodyA.angvel, rA);
        auto vB = bodyB.linvel + cross(bodyB.angvel, rB);
        auto relvel = vA - vB;
        auto normal_relspd = dot(relvel, normal);
        auto damper_force = cp.damping * -normal_relspd / num_points_in_same_plane;

        row.upper_limit = std::max(spring_force + damper_force, scalar(0)) * dt;

        auto &options = cache.get_options();
        options.error = -deflection / dt;
    }

    struct point_info {
        scalar angle;
        scalar half_length;
        scalar deflection;
        vector3 normal;
        vector3 pivot;
        scalar impulse;
        scalar friction;
        uint32_t lifetime;
    };

    std::array<point_info, max_contacts> infos;

    auto max_row_half_length = cyl.radius * scalar(0.9);
    auto r0_inv = scalar(1) / cyl.radius;
    auto num_points = size_t{};

    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);
        auto defl = std::max(-cp.distance, scalar(0));

        if (!(defl > EDYN_EPSILON)) {
            continue;
        }

        scalar angle = std::atan2(cp.pivotA.y, cp.pivotA.z);

        // Transform angle from [-π, π] to [0, 2π] because in spin space
        // angles are represented by a value in that range plus a number of
        // complete turns.
        if (angle < 0) {
            angle += 2 * pi;
        }

        // Add spin angle to bring the contact angle into spin space.
        angle += bodyA.spin_angle;

        auto &info = infos[num_points++];
        info.angle = angle;

        info.deflection = defl;
        info.half_length = std::min(scalar(0.4) * cyl.radius *
                                    (defl * r0_inv + scalar(2.25) *
                                    std::sqrt(defl * r0_inv)),
                                    max_row_half_length);
        info.normal = cp.normal;
        // Project A's pivot onto contact plane.
        auto pivotA_world = to_world_space(cp.pivotA, bodyA.origin, bodyA.orn);
        auto pivotB_world = to_world_space(cp.pivotB, bodyB.origin, bodyB.orn);
        info.pivot = project_plane(pivotA_world, pivotB_world, cp.normal);
        info.friction = cp.friction;
        info.lifetime = cp.lifetime;
        info.impulse = cp.normal_impulse;
    }

    if (num_points == 0) {
        num_patches = 0;
        return;
    }

    // Merge points together into a single patch based on their distance
    // along the circumference of the tire.
    for (unsigned i = 0; i < num_points; ++i) {
        auto &info_i = infos[i];
        auto min_i = normalize_angle(info_i.angle - info_i.half_length / cyl.radius);
        auto max_i = normalize_angle(info_i.angle + info_i.half_length / cyl.radius);
        unsigned patch_points = 1;

        auto weighted_angle = info_i.angle * info_i.deflection;
        auto weighted_normal = info_i.normal * info_i.deflection;
        auto weighted_pivot = info_i.pivot * info_i.deflection;
        auto defl_accum = info_i.deflection;

        // Look for nearby points ignoring the ones that were already
        // processed previously.
        for (unsigned k = i + 1; k < num_points;) {
            auto &info_k = infos[k];
            auto min_k = normalize_angle(info_k.angle - info_k.half_length / cyl.radius);
            auto max_k = normalize_angle(info_k.angle + info_k.half_length / cyl.radius);

            // Check if intervals intersect, considering they're in the [-π, π]
            // range and they wrap around.
            if (intervals_intersect_wrap_around(min_i, max_i, min_k, max_k, -pi, pi)) {
                weighted_angle += info_k.angle * info_k.deflection;
                weighted_normal += info_k.normal * info_k.deflection;
                weighted_pivot += info_k.pivot * info_k.deflection;
                defl_accum += info_k.deflection;
                info_i.impulse += info_k.impulse;
                info_i.friction += info_k.friction;
                info_i.lifetime = std::max(info_i.lifetime, info_k.lifetime);
                ++patch_points;

                // Remove k-th element by replacing with last and
                // decrementing size.
                info_k = infos[--num_points];
            } else {
                ++k;
            }
        }

        info_i.angle = weighted_angle / defl_accum;
        info_i.normal = normalize(weighted_normal / defl_accum);
        info_i.pivot = weighted_pivot / defl_accum;
        info_i.deflection = defl_accum / patch_points;
        info_i.friction /= patch_points;
    }

    // Look for an existing contact patch that is at about the same angle
    // as the newly calculated locations. Remove patches that are at locations
    // that do not have a close match among the new locations.

    // Keep track of which of the existing points have been merged into a patch
    // so they can be skipped later when creating new patches.
    auto merged_infos = make_array<max_contacts>(false);

    // Store the angle of each patch before assigning their new angle so the
    // delta can be calculated.
    std::array<scalar, max_contacts> prev_patch_angles;

    auto init_patch_with_info = [] (contact_patch_constraint::contact_patch &patch, const point_info &info) {
        patch.angle          = info.angle;
        patch.deflection     = info.deflection;
        patch.normal         = info.normal;
        patch.pivot          = info.pivot;
        patch.normal_impulse = info.impulse;
        patch.friction       = info.friction;
        patch.lifetime       = info.lifetime;
        patch.length         = info.half_length * 2;
    };

    for (unsigned i = 0; i < num_patches;) {
        auto &patch = patches[i];
        // Predict what is most likely to be the current angle of a patch
        // by subtracting the angle change over one step from the previous
        // angle.
        auto predicted_angle = normalize_angle(patch.angle - bodyA.spin * dt);

        bool found = false;

        for (unsigned j = 0; j < num_points; ++j) {
            auto &info = infos[j];

            // Consider wrap around.
            auto new_angle = normalize_angle(info.angle);
            auto a = std::min(predicted_angle, new_angle);
            auto b = std::max(predicted_angle, new_angle);
            auto dist = std::min(b - a, a + pi2 - b);

            if (dist < to_radians(5)) {
                prev_patch_angles[i] = patch.angle;
                init_patch_with_info(patch, info);
                merged_infos[j] = true;
                found = true;
                break;
            }
        }

        if (found) {
            ++i;
        } else {
            // Remove patch by assigning last and decrementing count.
            patch = patches[--num_patches];
        }
    }

    // Insert new contact patches.
    for (unsigned i = 0; i < num_points; ++i) {
        if (merged_infos[i]) continue;

        auto &info = infos[i];
        auto k = num_patches++;
        auto &patch = patches[k];
        prev_patch_angles[k] = patch.angle;
        init_patch_with_info(patch, info);
    }

    EDYN_ASSERT(num_patches > 0 && num_patches <= max_contacts);

    const auto &material = registry.get<tire_material>(body[0]);
    const auto sidewall_height = material.tire_radius - material.rim_radius;

    // Create constraint rows for each contact patch.
    for (unsigned i = 0; i < num_patches; ++i) {
        auto &patch = patches[i];

        const auto normal_force = patch.normal_impulse / dt;
        patch.applied_impulse.normal = patch.normal_impulse;

        const auto normal = patch.normal;
        auto sin_camber = std::clamp(dot(axis, normal), scalar(-1), scalar(1));
        auto camber_angle = std::asin(sin_camber);
        auto [lon_dir, lat_dir] = get_tire_directions(axis, normal, bodyA.orn);

        // Calculate contact patch width.
        // TODO: Variations in width require laterally interpolating tire
        // tread state over into new tread rows since they refer to different
        // sections of the contact patch along the lateral axis.
        auto normalized_contact_width = std::max(scalar(0.08), scalar(1) - scalar(1) /
            (normal_force * scalar(0.001) * (half_pi - std::abs(camber_angle)) / (std::abs(camber_angle) + scalar(0.001)) + 1));
        patch.width = cyl.half_length * 2 * normalized_contact_width;

        // Calculate starting point of contact patch on the contact plane.
        auto point_on_yz_plane = project_plane(patch.pivot, bodyA.origin, axis);
        auto radial_dir = normalize(point_on_yz_plane - bodyA.origin);
        auto axial_offset = axis * cyl.half_length * (sin_camber > 0 ? -1 : 1);
        auto circle_center = bodyA.origin + axial_offset;
        auto point_on_edge = circle_center + radial_dir * cyl.radius;

        bool is_parallel = std::abs(dot(radial_dir, normal)) < 0.001;
        std::array<vector3, 2> patch_lat_pos;
        auto patch_lat_deeper_index = sin_camber < 0 ? 1 : 0;

        if (is_parallel) {
            // The contact patch starting point is at the top of the
            // sidewall in this case.
            patch_lat_pos[patch_lat_deeper_index] = circle_center + radial_dir * (cyl.radius - sidewall_height);
        } else {
            // The starting point is at the intersection between the line
            // connecting the center of the cylinder cap face closest to the
            // contact plane and the support point along -normal with the
            // contact plane up to the height of the sidewall.
            auto min_fraction = scalar(1) - sidewall_height / cyl.radius ;
            auto fraction = dot(patch.pivot - circle_center, normal) / dot(radial_dir * cyl.radius, normal);
            fraction = std::clamp(fraction, min_fraction, scalar(1));
            patch_lat_pos[patch_lat_deeper_index] = circle_center + radial_dir * cyl.radius * fraction;
        }

        // Calculate the other end of the contact patch along the width of
        // the tire.
        auto patch_lat_other_index = patch_lat_deeper_index == 0 ? 1 : 0;
        patch_lat_pos[patch_lat_other_index] = patch_lat_pos[patch_lat_deeper_index] + lat_dir * patch.width * (sin_camber > 0 ? 1 : -1) * (dot(lat_dir, axis) > 0 ? 1 : -1);

        // Push the pivot downwards to match the height of the patch
        // position in case they differ in level.
        const auto patch_pivot_correction = normal * dot(normal, patch_lat_pos[patch_lat_deeper_index] - patch.pivot);
        patch.pivot += patch_pivot_correction;

        // Recalculate deflection in case it's been clamped since deformation
        // is limited by the sidewall height.
        patch.deflection = dot(normal, patch_lat_pos[patch_lat_deeper_index] - point_on_edge);
        EDYN_ASSERT(!(patch.deflection < 0));

        // Calculate center of pressure.
        auto normalized_center_offset = -std::sin(std::atan(camber_angle));
        auto center_lerp_param = (normalized_center_offset + scalar(1)) * scalar(0.5);
        auto contact_center = lerp(patch_lat_pos[0], patch_lat_pos[1], center_lerp_param);
        auto geometric_center = lerp(patch_lat_pos[0], patch_lat_pos[1], scalar(0.5));

        // Where the tread row starts in the x-axis in object space.
        const auto row_start = sin_camber > 0 ? -cyl.half_length : cyl.half_length - patch.width;

        // Calculate deflection on each side of the contact patch which will be
        // interpolated to find the deflection at each tread row.
        // Ensure deflection0 is the deflection at the row start, i.e.
        // at negative x in tire's object space.
        scalar deflection0, deflection1;

        if (sin_camber > 0) {
            deflection0 = patch.deflection;
            deflection1 = patch.deflection - sin_camber * patch.width;
        } else {
            deflection1 = patch.deflection;
            deflection0 = patch.deflection + sin_camber * patch.width;
        }

        // Make the smaller deflection at least 10% of the bigger deflection.
        auto min_defl = std::max(deflection0, deflection1) * scalar(0.1);
        deflection0 = std::max(deflection0, min_defl);
        deflection1 = std::max(deflection1, min_defl);

        const auto sin_contact_angle = std::sin(patch.angle);
        const auto cos_contact_angle = std::cos(patch.angle);

        // Accumulate forces and errors along all bristles.
        auto lon_force = scalar(0);
        auto lat_force = scalar(0);
        auto aligning_torque = scalar(0);
        auto tread_width = patch.width / num_tread_rows;
        auto normal_pressure = normal_force / (patch.width * patch.length);

        // Number of full turns since last update.
        auto spin_count_delta = bodyA.spin_count - patch.spin_count;
        // Calculate previous contact angle including the full turns so all
        // ranges and angles are all laid out in a segment without wrapping around.
        auto prev_contact_angle = prev_patch_angles[i] - spin_count_delta * pi2;
        auto num_sliding_bristles = 0;

        for (size_t row_idx = 0; row_idx < num_tread_rows; ++row_idx) {
            auto &tread_row = patch.tread_rows[row_idx];

            // The patch is divided in tread rows of equal width. Each row has a
            // different length, which is proportional to the deflection. Sample
            // the points along the middle of the row.
            auto row_x = row_start + tread_width * scalar(row_idx + 0.5);

            // Normal deflection and length for this row of bristles.
            auto row_fraction = (row_x - row_start) / patch.width;
            auto defl = lerp(deflection0, deflection1, row_fraction);

            if (defl < 0.0001) {
                // Reset tread and bristles.
                tread_row.half_length = 0;
                tread_row.half_angle = 0;

                // Set bristle properties according to the contact angle. This is
                // an empty range of the patch thus put all bristles at the same
                // location.
                auto row_start_pos_local = vector3{
                    row_x,
                    sin_contact_angle * cyl.radius,
                    cos_contact_angle * cyl.radius
                };
                auto row_start_pos = project_plane(to_world_space(row_start_pos_local, bodyA.origin, spin_ornA), geometric_center, normal);
                auto row_start_posB = to_object_space(row_start_pos, bodyB.pos, bodyB.orn);
                tread_row.start_posB = tread_row.end_posB = row_start_posB;

                for (auto &bristle : tread_row.bristles) {
                    bristle.tip = bristle.root = row_start_pos;
                    bristle.pivotA = row_start_pos_local;
                    bristle.pivotB = row_start_posB;
                }

                continue;
            }

            //auto max_row_half_length = cyl.radius * scalar(0.9);
            auto row_length = patch.length;
            auto row_half_length = row_length / scalar(2);
            auto row_half_angle = std::asin(row_half_length / cyl.radius);
            auto row_angle = scalar(2) * row_half_angle;

            auto bristle_angle_delta = row_angle / scalar(bristles_per_row);
            auto bristle_length_delta = row_length / scalar(bristles_per_row);

            // Contact patch extents in radians for this row.
            auto row_start_angle = patch.angle - row_half_angle;
            auto row_end_angle   = patch.angle + row_half_angle;

            auto prev_row_start_angle = prev_contact_angle - tread_row.half_angle;
            auto prev_row_end_angle   = prev_contact_angle + tread_row.half_angle;

            // Calculate intersection between previous and current contact row
            // range. Bristles that lie in the intersection will be matched to a
            // brush tip calculated as an interpolation of the previous values.
            // For the bristles that lie outside of the intersection, assign a
            // brush tip from the line connecting the start of the previous and
            // current tread rows for this row, which is where new bristles
            // are laid down as the tire rolls over the surface.
            auto intersection_start_angle = std::max(row_start_angle, prev_row_start_angle);
            auto intersection_end_angle = std::min(row_end_angle, prev_row_end_angle);
            auto intersects = intersection_start_angle < intersection_end_angle;

            // Midpoint of the tread row in spin space.
            auto row_mid_pos_local = vector3{row_x,
                                             sin_contact_angle * (cyl.radius - defl),
                                             cos_contact_angle * (cyl.radius - defl)};
            // Direction vector of tread row in spin space. It is the radial direction
            // rotated 90 degrees clockwise.
            auto row_dir_local = vector3{0, cos_contact_angle, -sin_contact_angle};

            auto row_start_pos_local = row_mid_pos_local - row_dir_local * row_half_length;
            auto row_end_pos_local = row_mid_pos_local + row_dir_local * row_half_length;
            auto row_start_pos = project_plane(to_world_space(row_start_pos_local, bodyA.origin, spin_ornA), geometric_center, normal);
            auto row_end_pos   = project_plane(to_world_space(row_end_pos_local, bodyA.origin, spin_ornA), geometric_center, normal);

            auto prev_row_start_pos = project_plane(to_world_space(tread_row.start_posB, bodyB.pos, bodyB.orn), geometric_center, normal);
            auto prev_row_end_pos   = project_plane(to_world_space(tread_row.end_posB, bodyB.pos, bodyB.orn), geometric_center, normal);

            const auto is_new_row = tread_row.half_length < EDYN_EPSILON;
            auto prev_bristle_defl = vector3_zero;
            auto prev_row_bristle_pivotB = std::array<vector3, contact_patch_constraint::bristles_per_row>{};
            auto prev_row_bristle_defl = std::array<vector3, contact_patch_constraint::bristles_per_row>{};

            // Values of `bristle.pivotB` (i.e. the object space position of the
            // bristle tip) will change during bristle updates. Store their previous
            // values here so they can be referenced when interpolating between
            // the previous bristle tips in the region where the previous and
            // current contact patches intersect.
            for (size_t i = 0; i < bristles_per_row; ++i) {
                auto &bristle = tread_row.bristles[i];
                prev_row_bristle_pivotB[i] = bristle.pivotB;
                prev_row_bristle_defl[i] = bristle.tip - bristle.root;
            }

            for (size_t bristle_idx = 0; bristle_idx < bristles_per_row; ++bristle_idx) {
                auto &bristle = tread_row.bristles[bristle_idx];

                // Calculate bristle root position in tire's space. It lies along the
                // tread row length if there's no camber. With non-zero camber, an offset
                // is added to account for the curvature of the tread row with respect
                // to the contact surface, which is modeled as a section of an ellipse
                // which is a semi-circle scaled down along the x axis by `sin_camber`,
                // i.e. `f(x) = sqrt(r^2 - x^2) * sin(camber)`. This ellipse can be
                // observed by looking into the tire tread circles from above along the
                // normal vector.
                auto bristle_root_fraction = scalar(bristle_idx + 0.5) / scalar(bristles_per_row);
                // Z position from center. Use it in the circle function.
                auto bristle_z_local = (bristle_root_fraction - scalar(0.5)) * row_length;
                // Also subtract the tread row effective radius times `sin_camber` to
                // remove the offset from the tread row segment.
                auto camber_offset = std::sqrt(cyl.radius * cyl.radius - bristle_z_local * bristle_z_local) * sin_camber - (cyl.radius - defl) * sin_camber;
                auto bristle_root = lerp(row_start_pos, row_end_pos, bristle_root_fraction) + lat_dir * camber_offset;
                bristle.pivotA = to_object_space(bristle_root, bodyA.origin, spin_ornA);

                // Calculate bristle tip position and relative velocity between
                // root and tip.
                // Note the 0.5 term which places the sampling point in the middle
                // of the rectangular tread area.
                auto bristle_angle = row_start_angle + bristle_angle_delta * scalar(bristle_idx + 0.5);
                auto bristle_tip = vector3_zero;
                auto bristle_defl0 = vector3_zero;
                auto bristle_lifetime_dt = dt;

                if (is_new_row) {
                    bristle_tip = bristle_root;
                    bristle.pivotB = to_object_space(bristle_tip, bodyB.pos, bodyB.orn);
                } else if (intersects && bristle_angle >= intersection_start_angle && bristle_angle <= intersection_end_angle) {
                    // Bristle lies in the intersection.
                    // Find index of bristles in the previous patch which surround this bristle.
                    auto fraction = (bristle_angle - prev_row_start_angle) /
                                    (prev_row_end_angle - prev_row_start_angle);
                    auto after_idx = static_cast<size_t>(std::round(fraction * bristles_per_row));
                    EDYN_ASSERT(after_idx <= bristles_per_row);

                    scalar before_angle, after_angle;
                    vector3 before_pivotB, after_pivotB;
                    vector3 before_defl, after_defl;

                    auto prev_bristle_angle_delta = (tread_row.half_angle * scalar(2)) / scalar(bristles_per_row);

                    if (after_idx == 0) {
                        // This bristle is located before the first bristle in the previous
                        // contact patch. Use the start of the previous patch as the location
                        // before it and the first bristle as the location after it.
                        before_angle = prev_row_start_angle;
                        after_angle = before_angle + prev_bristle_angle_delta * scalar(0.5);

                        before_pivotB = tread_row.start_posB; // This still holds the previous value.
                        after_pivotB = prev_row_bristle_pivotB[after_idx];

                        before_defl = vector3_zero;
                        after_defl = prev_row_bristle_defl.front();
                    } else if (after_idx == bristles_per_row) {
                        // This bristle is located after the last bristle in the previous
                        // contact patch. Use the last bristle as the location before it and
                        // the end of the previous contact patch as the location after it.
                        auto before_idx = after_idx - 1;
                        after_angle = prev_row_end_angle;
                        before_angle = after_angle - prev_bristle_angle_delta * scalar(0.5);

                        before_pivotB = prev_row_bristle_pivotB[before_idx];
                        after_pivotB = tread_row.end_posB; // This still holds the previous value.

                        before_defl = prev_row_bristle_defl.back();
                        after_defl = vector3_zero;
                    } else {
                        // This bristle lies between two of the bristles in the previous
                        // contact patch.
                        auto before_idx = after_idx - 1;
                        before_angle = prev_row_start_angle + prev_bristle_angle_delta * scalar(before_idx + 0.5);
                        after_angle = prev_row_start_angle + prev_bristle_angle_delta * scalar(after_idx + 0.5);

                        before_pivotB = prev_row_bristle_pivotB[before_idx];
                        after_pivotB = prev_row_bristle_pivotB[after_idx];

                        before_defl = prev_row_bristle_defl[before_idx];
                        after_defl = prev_row_bristle_defl[after_idx];
                    }

                    // Linearly interpolate the bristle tips.
                    auto inbetween_fraction = (bristle_angle - before_angle) / (after_angle - before_angle);
                    bristle.pivotB = lerp(before_pivotB, after_pivotB, inbetween_fraction);
                    bristle_tip = project_plane(to_world_space(bristle.pivotB, bodyB.pos, bodyB.orn), geometric_center, normal);

                    bristle_defl0 = lerp(before_defl, after_defl, inbetween_fraction);
                    bristle_lifetime_dt = dt;
                } else if (bristle_angle >= prev_row_end_angle) {
                    // Bristle is located after the end of the previous contact patch,
                    // which means it was laid down the contact plane along the way between
                    // the previous and current state. The exact location it was laid down
                    // lies along the line connecting the back of the previous row and the
                    // back of the current row.
                    auto fraction = (bristle_angle - prev_row_end_angle) /
                                    (row_end_angle - prev_row_end_angle);
                    bristle_tip = lerp(prev_row_end_pos, row_end_pos, fraction);
                    bristle.pivotB = to_object_space(bristle_tip, bodyB.pos, bodyB.orn);

                    bristle_defl0 = vector3_zero;
                    bristle_lifetime_dt = dt * (1 - fraction);
                } else if (bristle_angle <= prev_row_start_angle) {
                    // Bristle is located before the start of the previous contact patch.
                    // Place it along the line connecting the start position of the previous
                    // to the start position of the current contact patch.
                    auto fraction = (bristle_angle - prev_row_start_angle) /
                                    (row_start_angle - prev_row_start_angle);
                    bristle_tip = lerp(prev_row_start_pos, row_start_pos, fraction);
                    bristle.pivotB = to_object_space(bristle_tip, bodyB.pos, bodyB.orn);

                    bristle_defl0 = vector3_zero;
                    bristle_lifetime_dt = dt * (1 - fraction);
                } else {
                    EDYN_ASSERT(false);
                }

                auto mu0 = patch.friction * std::exp(scalar(-0.001) * material.load_sensitivity * normal_force);
                bristle.friction = mu0 / (1 + material.speed_sensitivity * bristle.sliding_spd);

                // The length for the first bristle is halved since it is located in
                // the middle of the rectangular tread.
                auto tread_length = (bristle_idx == 0 ? scalar(0.5) : scalar(1)) * bristle_length_delta;
                auto tread_area = tread_width * tread_length;

                // The force is calculated as an integral from the previous deflection until the
                // current deflection along the row.
                auto bristle_defl = bristle_tip - bristle_root;
                auto bristle_defl_len = length(bristle_defl);

                // Perform calculations with a scaling factor to avoid the large
                // numbers that result from using units such as N/m^2.
                const auto pressure_scaling = scalar(1e-6);
                const auto lon_tread_stiffness_scaled = material.lon_tread_stiffness * pressure_scaling;
                const auto lat_tread_stiffness_scaled = material.lat_tread_stiffness * pressure_scaling;
                const auto max_friction_pressure_scaled = bristle.friction * normal_pressure * pressure_scaling;

                auto bristle_pressure_scaled = std::sqrt(square(lon_tread_stiffness_scaled * dot(bristle_defl, lon_dir)) +
                                                         square(lat_tread_stiffness_scaled * dot(bristle_defl, lat_dir)));

                if (bristle_pressure_scaled < max_friction_pressure_scaled && bristle_defl_len < material.max_tread_deflection) {
                    bristle.sliding_spd = 0;
                } else if (bristle_defl_len > EDYN_EPSILON && max_friction_pressure_scaled > EDYN_EPSILON) {
                    // Bristle deflection force is greater than maximum friction force
                    // for the current normal load, which means the bristle must slide.
                    // Thus, move the bristle tip closer to its root so that the
                    // tangential deflection force is equals to the maximum friction force.
                    auto bristle_dir = bristle_defl / bristle_defl_len;
                    bristle_defl_len = max_friction_pressure_scaled /
                        std::sqrt(square(lon_tread_stiffness_scaled * dot(bristle_dir, lon_dir)) +
                                  square(lat_tread_stiffness_scaled * dot(bristle_dir, lat_dir)));
                    bristle_defl_len = std::min(bristle_defl_len, material.max_tread_deflection);
                    bristle_defl = bristle_dir * bristle_defl_len;

                    bristle_pressure_scaled = std::sqrt(square(lon_tread_stiffness_scaled * dot(bristle_defl, lon_dir)) +
                                                        square(lat_tread_stiffness_scaled * dot(bristle_defl, lat_dir)));
                    EDYN_ASSERT(bristle_pressure_scaled <= max_friction_pressure_scaled * 1.01);

                    auto bristle_tip_next = bristle_root + bristle_defl;
                    bristle.sliding_spd = distance(bristle_tip, bristle_tip_next) / bristle_lifetime_dt;
                    bristle_tip = bristle_tip_next;

                    ++num_sliding_bristles;
                }

                // Point of force application.
                auto prev_bristle_root = bristle_idx > 0 ?
                    tread_row.bristles[bristle_idx - 1].root :
                    row_start_pos;
                auto midpoint = (bristle_root + prev_bristle_root) * scalar(0.5);

                auto bristle_lon_force = material.lon_tread_stiffness * tread_area * dot(prev_bristle_defl + bristle_defl, lon_dir) * scalar(0.5);
                lon_force += bristle_lon_force;

                auto bristle_lat_force = material.lat_tread_stiffness * tread_area * dot(prev_bristle_defl + bristle_defl, lat_dir) * scalar(0.5);
                lat_force += bristle_lat_force;

                auto bristle_force = bristle_lon_force * lon_dir + bristle_lat_force * lat_dir;
                aligning_torque += dot(cross(midpoint - contact_center, bristle_force), normal);

                // Assign new root and tip.
                bristle.tip = bristle_tip;
                bristle.root = bristle_root;

                // Move pivot in B to match new tip location.
                bristle.pivotB = to_object_space(bristle_tip, bodyB.pos, bodyB.orn);

                prev_bristle_defl = bristle_defl;
            }

            // Add force from last bristle until end of patch.
            {
                auto tread_length = bristle_length_delta / scalar(2);
                auto tread_area = tread_width * tread_length;
                auto bristle_root = tread_row.bristles.back().root;
                auto midpoint = (row_end_pos + bristle_root) * scalar(0.5);

                auto bristle_lon_force = material.lon_tread_stiffness * tread_area * dot(prev_bristle_defl, lon_dir) * scalar(0.5);
                lon_force += bristle_lon_force;

                auto bristle_lat_force = material.lat_tread_stiffness * tread_area * dot(prev_bristle_defl, lat_dir) * scalar(0.5);
                lat_force += bristle_lat_force;

                auto bristle_force = bristle_lon_force * lon_dir + bristle_lat_force * lat_dir;
                aligning_torque += dot(cross(midpoint - contact_center, bristle_force), normal);
            }

            tread_row.half_angle = row_half_angle;
            tread_row.half_length = row_half_length;
            tread_row.start_posB = to_object_space(row_start_pos, bodyB.pos, bodyB.orn);
            tread_row.end_posB = to_object_space(row_end_pos, bodyB.pos, bodyB.orn);
            tread_row.start_pos = row_start_pos;
            tread_row.end_pos = row_end_pos;
        }

        // Calculate average bristle sliding speed.
        patch.sliding_spd_avg = scalar(0);

        for (auto &row : patch.tread_rows) {
            for (auto &bristle : row.bristles) {
                patch.sliding_spd_avg += bristle.sliding_spd;
            }
        }

        constexpr auto num_bristles_inv = scalar(1) / scalar(num_tread_rows * bristles_per_row);
        patch.sliding_spd_avg *= num_bristles_inv;
        patch.sliding_ratio = scalar(num_sliding_bristles) * num_bristles_inv;
        auto rA = contact_center - bodyA.pos;
        auto rB = contact_center - bodyB.pos;

        // Longitudinal stiffness.
        {
            auto p = cross(rA, lon_dir);
            auto q = cross(rB, lon_dir);
            auto spring_impulse = lon_force * dt;

            auto &row = cache.add_row_with_spin();
            row.J = {lon_dir, p, -lon_dir, -q};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.impulse = patch.applied_impulse.longitudinal;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = spin_axisA;
            row.spin_axis[1] = spin_axisB;

            auto &options = cache.get_options();
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        }

        // Lateral stiffness.
        {
            auto p = cross(rA, lat_dir);
            auto q = cross(rB, lat_dir);
            auto spring_impulse = lat_force * dt;

            auto &row = cache.add_row();
            row.J = {lat_dir, p, -lat_dir, -q};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.impulse = patch.applied_impulse.lateral;

            auto &options = cache.get_options();
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        }

        // Aligning moment.
        {
            auto spring_impulse = aligning_torque * dt;

            auto &row = cache.add_row();
            row.J = {vector3_zero, normal, vector3_zero, -normal};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.impulse = patch.applied_impulse.aligning;

            auto &options = cache.get_options();
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        }

        patch.sin_camber = sin_camber;
        patch.center = geometric_center;
        patch.pivot = contact_center;
        patch.lat_dir = lat_dir;
        patch.lon_dir = lon_dir;
        patch.spin_count = bodyA.spin_count;
    }
}

void contact_patch_constraint::store_applied_impulses(const std::vector<scalar> &impulses, contact_manifold &manifold) {
    int row_idx = 0;

    manifold.each_point([&](contact_point &cp) {
        cp.normal_impulse = impulses[row_idx++];
    });

    for (unsigned i = 0; i < num_patches; ++i) {
        auto &patch = patches[i];
        patch.applied_impulse.longitudinal = impulses[row_idx++];
        patch.applied_impulse.lateral = impulses[row_idx++];
        patch.applied_impulse.aligning = impulses[row_idx++];
    }
}

}
