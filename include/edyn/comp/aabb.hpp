#ifndef EDYN_COMP_AABB_HPP
#define EDYN_COMP_AABB_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief Axis-aligned bounding box.
 */
struct AABB {
    vector3 min;
    vector3 max;

	inline AABB inset(const vector3 &v) const {
		return {min + v, max - v};
	}
};

inline bool intersect(const AABB &b0, const AABB &b1) {
    return (b0.min.x <= b1.max.x) &&
		   (b0.max.x >= b1.min.x) &&
		   (b0.min.y <= b1.max.y) &&
		   (b0.max.y >= b1.min.y) &&
		   (b0.min.z <= b1.max.z) &&
		   (b0.max.z >= b1.min.z);
}

}

#endif // EDYN_COMP_AABB_HPP