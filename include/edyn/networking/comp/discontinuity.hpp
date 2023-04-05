#ifndef EDYN_NETWORKING_COMP_DISCONTINUITY_HPP
#define EDYN_NETWORKING_COMP_DISCONTINUITY_HPP

#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief A discontinuity introduced by misprediction in the client-side. The
 * offsets are added to the present position and orientation to ameliorate
 * visual disturbances caused by snapping the physics state to the
 * extrapolation result.
 */
struct discontinuity {
    vector3 position_offset {vector3_zero};
    quaternion orientation_offset {quaternion_identity};
};

struct discontinuity_accumulator : public discontinuity {};

inline void merge_component(discontinuity_accumulator &component, const discontinuity_accumulator &new_value) {
    component.position_offset += new_value.position_offset;
    component.orientation_offset = edyn::normalize(component.orientation_offset * new_value.orientation_offset);
}

struct previous_position : public vector3 {
    previous_position & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

struct previous_orientation : public quaternion {
    previous_orientation & operator=(const quaternion &q) {
        quaternion::operator=(q);
        return *this;
    }
};

}

#endif // EDYN_NETWORKING_COMP_DISCONTINUITY_HPP
