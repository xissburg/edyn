#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx) {
    return {};
}

collision_result collide(const cylinder_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
