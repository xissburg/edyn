#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return {};
}

collision_result collide(const polyhedron_shape &shA, const box_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
