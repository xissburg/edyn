#ifndef EDYN_COMP_AABB_HPP
#define EDYN_COMP_AABB_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"

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
    return intersect_aabb(b0.min, b0.max, b1.min, b1.max);
}

}

#endif // EDYN_COMP_AABB_HPP