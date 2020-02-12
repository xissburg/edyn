#ifndef EDYN_UTIL_SPRING_UTIL_HPP
#define EDYN_UTIL_SPRING_UTIL_HPP

#include "edyn/math/linear_curve.hpp"

namespace edyn {

linear_curve spring_stiffness_curve(scalar primary_stiffness, scalar primary_max_defl,
                                    scalar secondary_stiffness, scalar secondary_max_defl);

scalar spring_preload(const linear_curve& curve, 
                      scalar piston_rod_length,
                      scalar damper_body_length,
                      scalar damper_body_offset,
                      scalar spring_offset,
                      scalar spring_perch_offset,
                      scalar spring_divider_length,
                      scalar spring_rest_length,
                      scalar secondary_spring_rest_length);

}

#endif // EDYN_UTIL_SPRING_UTIL_HPP