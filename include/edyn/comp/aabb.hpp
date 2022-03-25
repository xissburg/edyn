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

    inline vector3 center() const {
        return (min + max) * scalar(0.5);
    }

    // Returns this AABB's surface area.
    inline scalar area() const {
        auto d = max - min;
        return scalar{2} * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    // Returns whether this AABB contains point `p`.
    bool contains(const vector3 &p) const {
        return min <= p && p <= max;
    }

    // Returns whether `aabb` is contained within this AABB.
    bool contains(const AABB &aabb) const {
        return contains(aabb.min) && contains(aabb.max);
    }
};

inline bool intersect(const AABB &b0, const AABB &b1) {
    return intersect_aabb(b0.min, b0.max, b1.min, b1.max);
}

inline AABB enclosing_aabb(const AABB &b0, const AABB &b1) {
    return {
        min(b0.min, b1.min),
        max(b0.max, b1.max)
    };
}

template<typename Archive>
void serialize(Archive &archive, AABB &aabb) {
    archive(aabb.min);
    archive(aabb.max);
}

}

#endif // EDYN_COMP_AABB_HPP
