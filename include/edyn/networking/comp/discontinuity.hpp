#ifndef EDYN_NETWORKING_COMP_DISCONTINUITY_HPP
#define EDYN_NETWORKING_COMP_DISCONTINUITY_HPP

#include "edyn/comp/spin.hpp"
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

struct discontinuity_spin {
    scalar offset {};
};

struct discontinuity_spin_accumulator : public discontinuity_spin {};

inline void merge_component(discontinuity_spin_accumulator &component, const discontinuity_spin_accumulator &new_value) {
    component.offset += new_value.offset;
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

struct previous_spin_angle : public spin_angle {
    previous_spin_angle & operator=(const spin_angle &s) {
        spin_angle::operator=(s);
        return *this;
    }
};

}

#endif // EDYN_NETWORKING_COMP_DISCONTINUITY_HPP
