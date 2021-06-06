#ifndef EDYN_COLLISION_SHAPE_FEATURE_HPP
#define EDYN_COLLISION_SHAPE_FEATURE_HPP

#include <cstdint>
#include <variant>
#include "edyn/shapes/plane_shape.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/mesh_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/paged_mesh_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/util/triangle_util.hpp"

namespace edyn {

struct shape_feature {
    std::variant<
        box_feature,
        cylinder_feature,
        capsule_feature,
        polyhedron_feature,
        triangle_feature
    > feature;
    uint64_t index;
};

}

#endif // EDYN_COLLISION_SHAPE_FEATURE_HPP
