#ifndef EDYN_NETWORKING_COMP_DISCONTINUITY_HPP
#define EDYN_NETWORKING_COMP_DISCONTINUITY_HPP

#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief A discontinuity introduced by misprediction in the client-side. The
 * offsets are added to the present position and orientation to ammeliorate
 * visual disturbances caused by snapping the physics state to the
 * extrapolation result.
 */
struct discontinuity {
    vector3 position_offset {vector3_zero};
    quaternion orientation_offset {quaternion_identity};
};

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
