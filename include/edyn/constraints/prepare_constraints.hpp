#ifndef EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
#define EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

struct row_cache;

template<typename C>
void prepare_constraints(entt::registry &, row_cache &, scalar dt) {}

template<typename C>
void iterate_constraints(entt::registry &, row_cache &, scalar dt) {}

}

#endif // EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
