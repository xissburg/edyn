#ifndef EDYN_COMP_CONSTRAINT_HPP
#define EDYN_COMP_CONSTRAINT_HPP

#include <variant>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/constants.hpp"
#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/constraints/point_constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

using constrained_entities = std::array<entt::entity, max_constrained_entities>;

struct constraint {
    constrained_entities body = 
        make_array<max_constrained_entities>(entt::entity{entt::null});

    std::variant<contact_constraint, 
                 point_constraint, 
                 distance_constraint,
                 soft_distance_constraint,
                 hinge_constraint,
                 generic_constraint> var;

    std::array<scalar, max_constraint_rows> impulse;
};

}

#endif // EDYN_COMP_CONSTRAINT_HPP