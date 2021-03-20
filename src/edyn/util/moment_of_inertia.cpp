#include "edyn/util/moment_of_inertia.hpp"

namespace edyn {

vector3 moment_of_inertia_solid_box(scalar mass, const vector3 &extents) {
    return scalar(1) / scalar(12) * mass * vector3{
        extents.y * extents.y + extents.z * extents.z,
        extents.z * extents.z + extents.x * extents.x,
        extents.x * extents.x + extents.y * extents.y,
    };
}

vector3 moment_of_inertia_solid_capsule(scalar mass, scalar len, scalar radius) {
    auto xx = scalar(0.5) * mass * radius * radius;
    auto yy_zz = mass * (scalar(1) / scalar(12) * (scalar(3) * radius * radius + len * len) +
                    scalar(0.4) * radius * radius + 
                    scalar(0.375) * radius * len + 
                    scalar(0.25) * len * len);
    return {xx, yy_zz, yy_zz};
}

vector3 moment_of_inertia_solid_sphere(scalar mass, scalar radius) {
    auto s = scalar(0.4) * mass * radius * radius;
    return {s, s, s};
}

vector3 moment_of_inertia_hollow_sphere(scalar mass, scalar radius) {
    auto s = scalar(2) / scalar(3) * mass * radius * radius;
    return {s, s, s};
}

vector3 moment_of_inertia_solid_cylinder(scalar mass, scalar len, scalar radius) {
    scalar xx = scalar(0.5) * mass * radius * radius;
    scalar yy_zz =  scalar(1) / scalar(12) * mass * (scalar(3) * radius * radius + len * len);
    return {xx, yy_zz, yy_zz};
}

vector3 moment_of_inertia_hollow_cylinder(scalar mass, scalar len, 
                                          scalar inner_radius, scalar outer_radius) {
    auto rr = inner_radius * inner_radius + outer_radius * outer_radius;
    auto xx = scalar(0.5) * mass * rr;
    auto yy_zz = scalar(1) / scalar(12) * mass * (scalar(3) * rr + len * len);
    return {xx, yy_zz, yy_zz};
}

}
