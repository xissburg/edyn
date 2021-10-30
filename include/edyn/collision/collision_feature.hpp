#ifndef EDYN_COLLISION_COLLISION_FEATURE_HPP
#define EDYN_COLLISION_COLLISION_FEATURE_HPP

#include "edyn/shapes/shapes.hpp"

namespace edyn {

struct collision_feature {
    // Feature type.
    shape_feature_t feature;

    // Feature index.
    size_t index;

    // Part where the feature is at. Only used by shapes which are made of
    // individual parts, such as compound shape (child shape index) and
    // paged mesh shape (triangle mesh index).
    size_t part;
};

}

#endif // EDYN_COLLISION_COLLISION_FEATURE_HPP
