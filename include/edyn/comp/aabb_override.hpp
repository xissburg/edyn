#ifndef EDYN_COMP_AABB_OVERRIDE_HPP
#define EDYN_COMP_AABB_OVERRIDE_HPP

#include "edyn/comp/aabb.hpp"

namespace edyn {

/**
 * @brief Overrides the AABB of a rigid body. The AABB of the shape assigned to
 * that rigid body is ignored in broadphase. This AABB is used instead.
 */
struct AABB_override : public AABB {
    AABB_override & operator=(const AABB &a) {
        AABB::operator=(a);
        return *this;
    }
};

}

#endif // EDYN_COMP_AABB_OVERRIDE_HPP
