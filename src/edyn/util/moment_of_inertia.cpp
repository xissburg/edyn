#include "edyn/util/moment_of_inertia.hpp"
#include "edyn/math/vector3.hpp"
#include <variant>

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

matrix3x3 moment_of_inertia_polyhedron(scalar mass,
                                       const std::vector<vector3> &vertices, 
                                       const std::vector<uint16_t> &indices,
                                       const std::vector<uint16_t> &faces) {
    // Reference: 
    // https://github.com/erich666/jgt-code/blob/master/Volume_11/Number_2/Kallay2006/Moment_of_Inertia.cpp
    scalar volume = 0;
    auto center = vector3_zero;
    scalar xx = 0;
    scalar yy = 0;
    scalar zz = 0;
    scalar yz = 0;
    scalar zx = 0;
    scalar xy = 0;

    EDYN_ASSERT(faces.size() % 2 == 0);
    auto num_faces = faces.size() / 2;

    for (size_t i = 0; i < num_faces; ++i) {
        auto first = faces[i * 2];
        auto count = faces[i * 2 + 1];

        auto i0 = indices[first];
        auto &v0 = vertices[i0];

        for (size_t j = 1; j < count - 1; ++j) {
            auto i1 = indices[first + j];
            auto i2 = indices[first + j + 1];
            auto &v1 = vertices[i1];
            auto &v2 = vertices[i2];

            // Parallelepiped volume. Tetrahedron volume is 1/6th of it.
            auto pd_vol = triple_product(v0, v1, v2);

            // Contribution to the mass.
            volume += pd_vol;

            // Contribution to the centroid.
            auto v3 = v0 + v1 + v2;
            center += pd_vol * v3;

            // Contribution to the moment of inertia monomials.
            xx += pd_vol * (v0.x * v0.x + v1.x * v1.x + v2.x * v2.x + v3.x * v3.x);
            yy += pd_vol * (v0.y * v0.y + v1.y * v1.y + v2.y * v2.y + v3.y * v3.y);
            zz += pd_vol * (v0.z * v0.z + v1.z * v1.z + v2.z * v2.z + v3.z * v3.z);
            yz += pd_vol * (v0.y * v0.z + v1.y * v1.z + v2.y * v2.z + v3.y * v3.z);
            zx += pd_vol * (v0.z * v0.x + v1.z * v1.x + v2.z * v2.x + v3.z * v3.x);
            xy += pd_vol * (v0.x * v0.y + v1.x * v1.y + v2.x * v2.y + v3.x * v3.y);
        }
    }

    auto density = mass / (volume / scalar(6));
    auto r = density / scalar (120);
    auto Iyz = yz * r;
    auto Izx = zx * r;
    auto Ixy = xy * r;
    auto Ixx = (yy + zz) * r;
    auto Iyy = (zz + xx) * r;
    auto Izz = (xx + yy) * r;

    return {
        vector3{Ixx, Ixy, Izx},
        vector3{Ixy, Iyy, Iyz},
        vector3{Izx, Iyz, Izz}
    };
}

matrix3x3 moment_of_inertia(const plane_shape &sh, scalar mass) {
    return diagonal_matrix(vector3_max);
}

matrix3x3 moment_of_inertia(const sphere_shape &sh, scalar mass) {
    return diagonal_matrix(moment_of_inertia_solid_sphere(mass, sh.radius));
}

matrix3x3 moment_of_inertia(const cylinder_shape &sh, scalar mass) {
    return diagonal_matrix(moment_of_inertia_solid_cylinder(mass, sh.half_length * 2, sh.radius));
}

matrix3x3 moment_of_inertia(const capsule_shape &sh, scalar mass) {
    return diagonal_matrix(moment_of_inertia_solid_capsule(mass, sh.half_length * 2, sh.radius));
}

matrix3x3 moment_of_inertia(const mesh_shape &sh, scalar mass) {
    return diagonal_matrix(vector3_max);
}

matrix3x3 moment_of_inertia(const box_shape &sh, scalar mass) {
    return diagonal_matrix(moment_of_inertia_solid_box(mass, sh.half_extents * 2));
}

matrix3x3 moment_of_inertia(const polyhedron_shape &sh, scalar mass) {
    return moment_of_inertia_polyhedron(mass, sh.mesh->vertices, sh.mesh->indices, sh.mesh->faces);
}

matrix3x3 moment_of_inertia(const paged_mesh_shape &sh, scalar mass) {
    return diagonal_matrix(vector3_max);
}

matrix3x3 moment_of_inertia(const shape &sh, scalar mass) {
    matrix3x3 inertia;
    std::visit([&] (auto &&s) {
        inertia = moment_of_inertia(s, mass);
    }, sh.var);
    return inertia;
}

}
