#ifndef EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT
#define EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct doublewishbone_constraint : public constraint_base {
    vector3 upper_pivotA;
    vector3 upper_pivotB;
    scalar upper_length;
    vector3 lower_pivotA;
    vector3 lower_pivotB;
    scalar lower_length;
    bool steerable;

    static const auto num_rows = 6;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});
};


template<>
void prepare_constraints<doublewishbone_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT
