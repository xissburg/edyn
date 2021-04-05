#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx) {
    // Convex polyhedron against cylinder SAT.
    // Calculate collision with polyhedron in the origin for better floating point
    // precision. Position of cylinder is modified accordingly.
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    // The pre-rotated vertices and normals are used to avoid rotating vertices
    // every time.
    auto &rmeshA = *(*ctx.rmeshA);

    scalar max_distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    scalar projection_cyl = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;



    if (max_distance > threshold) {
        return {};
    }

    

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis);

    return result;
}

collision_result collide(const cylinder_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
