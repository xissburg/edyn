#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto cyl_axis = rotate(ornA, vector3_x);
    vector3 disc_pos[] = {posA - cyl_axis * shA.length / 2, posA + cyl_axis * shA.length / 2};
    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);
    auto result = collision_result{};

    auto n_proj = normal - cyl_axis * dot(normal, cyl_axis);
    auto n_proj_len2 = length2(n_proj);

    if (n_proj_len2 > scalar(1e-7)) {
        n_proj /= std::sqrt(n_proj_len2);
        auto r = n_proj * shA.radius;

        for (int i = 0; i < 2; ++i) {
            auto disc_center_dist = dot(disc_pos[i] - center, normal);
            // Project the negated plane normal onto disc and scale it to make its
            // length equals to the disc radius to obtain the closest point to the
            // plane.

            auto p = disc_pos[i] - r;
            auto dist = dot(p - center, normal);

            if (dist < threshold) {
                auto idx = result.num_points % max_contacts;
                ++result.num_points;

                result.point[idx].pivotA = rotate(conjugate(ornA), p - posA);
                result.point[idx].pivotB = rotate(conjugate(ornB), p - normal * dist - posB);
                result.point[idx].normalB = shB.normal;
                result.point[idx].distance = dist;

                // If the center of the disc is also close to the plane, add 
                // points to the sides.
                if (disc_center_dist < threshold) {
                    // Rotate `r` by 90 degrees along the cylinder axis.
                    {
                        auto idx = result.num_points % max_contacts;
                        ++result.num_points;

                        auto rot90 = quaternion_axis_angle(cyl_axis, pi / 2);
                        auto p0 = disc_pos[i] + rotate(rot90, r);
                        result.point[idx].pivotA = rotate(conjugate(ornA), p0 - posA);
                        result.point[idx].pivotB = rotate(conjugate(ornB), p0 - normal * disc_center_dist - posB);
                        result.point[idx].normalB = shB.normal;
                        result.point[idx].distance = disc_center_dist;
                    }

                    // Rotate `r` by -90 degrees along the cylinder axis.
                    {
                        auto idx = result.num_points % max_contacts;
                        ++result.num_points;

                        auto rot90 = quaternion_axis_angle(cyl_axis, -pi / 2);
                        auto p0 = disc_pos[i] + rotate(rot90, r);
                        result.point[idx].pivotA = rotate(conjugate(ornA), p0 - posA);
                        result.point[idx].pivotB = rotate(conjugate(ornB), p0 - normal * disc_center_dist - posB);
                        result.point[idx].normalB = shB.normal;
                        result.point[idx].distance = disc_center_dist;
                    }

                    // If the point opposite to the closest is also close to the 
                    // plane, add it to the result.
                    auto p1 = disc_pos[i] + r;
                    auto dist1 = dot(p1 - center, normal);

                    if (dist1 < threshold) {
                        auto idx = result.num_points % max_contacts;
                        ++result.num_points;

                        result.point[idx].pivotA = rotate(conjugate(ornA), p1 - posA);
                        result.point[idx].pivotB = rotate(conjugate(ornB), p1 - normal * dist1 - posB);
                        result.point[idx].normalB = shB.normal;
                        result.point[idx].distance = dist1;
                    }
                }
            }
        }
    } else {
        // Disc and plane are parallel. Find closest disc and add 
        // multiple points on the perimeter of the disc.
        auto disc_center_dist0 = dot(disc_pos[0] - center, normal);
        auto disc_center_dist1 = dot(disc_pos[1] - center, normal);
        scalar side = disc_center_dist0 < disc_center_dist1 ? -1 : 1;

        result.num_points = 4;

        result.point[0].pivotA = {side * shA.length / 2, shA.radius, 0};
        result.point[1].pivotA = {side * shA.length / 2, -shA.radius, 0};
        result.point[2].pivotA = {side * shA.length / 2, 0, shA.radius};
        result.point[3].pivotA = {side * shA.length / 2, 0, -shA.radius};

        for (int k = 0; k < 4; ++k) {
            auto p_world = posA + rotate(ornA, result.point[k].pivotA);
            auto p_proj = p_world - normal * dot(p_world - center, normal);
            result.point[k].pivotB = rotate(conjugate(ornB), p_proj - posB);
            result.point[k].normalB = shB.normal;
            result.point[k].distance = std::min(disc_center_dist0, disc_center_dist1);
        }
    }
    
    return result;
}

}