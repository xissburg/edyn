#ifndef EDYN_DYNAMICS_SOLVER_STAGE_HPP
#define EDYN_DYNAMICS_SOLVER_STAGE_HPP

#include <cstdint>
#include <type_traits>

namespace edyn {

enum class solver_stage : uint8_t {
    init,
    prepare,
    iteration,
};

template<solver_stage value>
using solver_stage_value_t = std::integral_constant<solver_stage, value>;

}

#endif // EDYN_DYNAMICS_SOLVER_STAGE_HPP