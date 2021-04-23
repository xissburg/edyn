#include "edyn/util/shape_volume.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"

namespace edyn {

scalar cylinder_volume(scalar radius, scalar length) {
    return pi * radius * radius * length;
}

scalar sphere_volume(scalar radius) {
    return pi * radius * radius * radius * scalar(4) / scalar(3);
}

scalar box_volume(const vector3 &extents) {
    return extents.x * extents.y * extents.z;
}

scalar mesh_volume(const convex_mesh &mesh) {
    // Accumulate signed tetrahedron volumes, where each tetrahedron is formed
    // using a triangular face and the origin. This allows correct volume 
    // calculation even if the shape does not contain the origin.
    auto volume = scalar(0);

    EDYN_ASSERT(mesh.faces.size() % 2 == 0);
    auto num_faces = mesh.faces.size() / 2;

    for (size_t i = 0; i < num_faces; ++i) {
        auto first = mesh.faces[i * 2];
        auto count = mesh.faces[i * 2 + 1];

        auto i0 = mesh.indices[first];
        auto &v0 = mesh.vertices[i0];
        auto &normal = mesh.normals[i];

        for (size_t j = 1; j < count - 1; ++j) {
            auto i1 = mesh.indices[first + j];
            auto i2 = mesh.indices[first + j + 1];
            auto &v1 = mesh.vertices[i1];
            auto &v2 = mesh.vertices[i2];
            auto area = length(cross(v1 - v0, v2 - v1)) * scalar(0.5);
            auto height = dot(v0, normal);
            // Signed tetrahedron volume.
            auto tet_vol = area * height / 3;
            volume += tet_vol;
        }
    }

    return volume;
}

scalar shape_volume(const box_shape &sh) {
    return box_volume(sh.half_extents * 2);
}

scalar shape_volume(const capsule_shape &sh) {
    return cylinder_volume(sh.radius, sh.half_length * 2) + sphere_volume(sh.radius);
}

scalar shape_volume(const compound_shape &sh) {
    auto volume = scalar(0);

    for (auto &node : sh.nodes) {
        std::visit([&] (auto &&s) {
            volume += shape_volume(s);
        }, node.var);
    }

    return volume;
}

scalar shape_volume(const cylinder_shape &sh) {
    return cylinder_volume(sh.radius, sh.half_length * 2);
}

scalar shape_volume(const polyhedron_shape &sh) {
    return mesh_volume(*sh.mesh);
}

scalar shape_volume(const sphere_shape &sh) {
    return sphere_volume(sh.radius);
}

}