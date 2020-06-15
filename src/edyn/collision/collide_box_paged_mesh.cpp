#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};
    return result;
}

}