#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const capsule_shape &shA, const cylinder_shape &shB, 
                         const collision_context &ctx) {
    // TODO
    return {};
}

collision_result collide(const cylinder_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
