#ifndef EDYN_DYNAMICS_SOLVER_STAGE_HPP
#define EDYN_DYNAMICS_SOLVER_STAGE_HPP

#include <cstdint>

namespace edyn {

enum class solver_stage : uint8_t {
    init,
    prepare,
    solve_row,
};

}

#endif // EDYN_DYNAMICS_SOLVER_STAGE_HPP