#include "edyn/util/spring_util.hpp"

namespace edyn {

// TODO: Include bump stop stiffness in curve. ****
linear_curve spring_stiffness_curve(scalar primary_stiffness, scalar primary_max_defl,
                                    scalar secondary_stiffness, scalar secondary_max_defl) {
    linear_curve curve;
    curve.add(-1, 0);
    curve.add(0, 0);

    if (secondary_stiffness > 0) {
        auto combined_stiffness = primary_stiffness * secondary_stiffness / (primary_stiffness + secondary_stiffness);
        // Find primary deflection when secondary deflection is at maximum.
        auto primary_defl = (secondary_stiffness - combined_stiffness) * secondary_max_defl / combined_stiffness;
        auto transition_pt1 = primary_defl + secondary_max_defl;
        curve.add(transition_pt1, transition_pt1 * combined_stiffness);

        auto transition_pt2 = transition_pt1 + scalar(0.005);
        curve.add(transition_pt2, (transition_pt2 - secondary_max_defl) * primary_stiffness);

        auto max_defl = primary_max_defl + secondary_max_defl;
        curve.add(max_defl, primary_max_defl * primary_stiffness);
        curve.add(max_defl + 0.01, 1e6);
        curve.add(max_defl + 1, 1e6);
    } else {
        curve.add(primary_max_defl, primary_max_defl * primary_stiffness);
        curve.add(primary_max_defl + 0.01, 1e6);
        curve.add(primary_max_defl + 1, 1e6);
    }

    return curve;
}

scalar spring_preload(const linear_curve& curve, 
                      scalar piston_rod_length,
                      scalar damper_body_length,
                      scalar damper_body_offset,
                      scalar spring_offset,
                      scalar spring_perch_offset,
                      scalar spring_divider_length,
                      scalar spring_rest_length,
                      scalar secondary_spring_rest_length) {
    scalar max_len = piston_rod_length + damper_body_length + damper_body_offset;
    max_len -= spring_offset + spring_perch_offset + damper_body_offset + spring_divider_length;
    scalar defl = spring_rest_length + secondary_spring_rest_length - max_len;
    return curve.get(defl);
}

}