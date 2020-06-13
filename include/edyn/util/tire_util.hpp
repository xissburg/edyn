#ifndef EDYN_UTIL_TIRE_UTIL_HPP
#define EDYN_UTIL_TIRE_UTIL_HPP

#include "edyn/math/math.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness, 
                                             scalar speed, 
                                             scalar inflation_pressure = 200000);
/*
vector3 calculate_tire_contact_center(vector3 &pos, quaternion &orn, vector3 &axis, 
                                      scalar radius, scalar half_length, vector3 &normal,
                                      vector3 &contact_pos, scalar deflection) {
    auto axis_hl = axis * half_length;
    auto sin_camber = dot(axis, normal);
    auto camber_angle = std::asin(sin_camber);
    auto normalized_contact_width = std::cos(std::atan(std::pow(std::abs(camber_angle), std::log(deflection * 300 + 1))));
    auto contact_width = half_length * 2 * normalized_contact_width;

    // Calculate center of pressure.
    auto normalized_center_offset = -std::sin(std::atan(camber_angle));

    // Where the row starts in the x-axis in object space.
    auto row_start = sin_camber < 0 ? -half_length : half_length - contact_width;

    // Intersect lines going from the circle center to the support point with the
    // contact plane to find the initial contact extent.
    auto circle_center0 = pos - axis_hl;
    auto circle_center1 = pos + axis_hl;
    auto sup0 = support_point_circle(pos - axis_hl, orn, radius, -normal);
    auto sup1 = sup0 + axis_hl * 2; // because circles are parallel
    auto intersection0 = intersect_line_plane(circle_center0, sup0 - circle_center0, contact_pos, normal);
    auto intersection1 = intersect_line_plane(circle_center1, sup1 - circle_center1, contact_pos, normal);
    auto cyl_len_inv = scalar(1) / (scalar(2) * half_length);
    auto plane_point0 = lerp(intersection0, intersection1, (row_start + half_length) * cyl_len_inv);
    auto plane_point1 = lerp(intersection0, intersection1, (row_start + half_length + contact_width) * cyl_len_inv);
    auto center_lerp_param = (normalized_center_offset + scalar(1)) * scalar(0.5);
    auto contact_center = lerp(plane_point0, plane_point1, center_lerp_param);

    return contact_center;
}*/

}

#endif // EDYN_UTIL_TIRE_UTIL_HPP