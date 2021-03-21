#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};
    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);

    for (size_t i = 0; i < shA.mesh->vertices.size(); ++i) {
        auto vertex_local = shA.mesh->vertices[i];
        auto vertex_world = posA + rotate(ornA, vertex_local);
        auto distance = dot(vertex_world - center, normal);

        if (distance > threshold) continue;

        auto pt_proj = vertex_world - normal * distance;    
        auto pivotB = rotate(conjugate(ornB), pt_proj - posB);
        result.maybe_add_point({vertex_local, pivotB, shB.normal, distance});
    }

    return result;
}

}
