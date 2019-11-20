#ifndef EDYN_COMP_CONSTRAINT_HPP
#define EDYN_COMP_CONSTRAINT_HPP

#include <variant>
#include <entt/entt.hpp>
#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/point_constraint.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

inline constexpr size_t max_constraint_rows = 8;

struct constraint {
    std::variant<point_constraint> var;
    std::array<entt::entity, max_constraint_rows> row = 
        make_array<max_constraint_rows, entt::entity>(entt::null);
};

}

#endif // EDYN_COMP_CONSTRAINT_HPP