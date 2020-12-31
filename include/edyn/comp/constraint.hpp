#ifndef EDYN_COMP_CONSTRAINT_HPP
#define EDYN_COMP_CONSTRAINT_HPP

#include <variant>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/constraints/point_constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/antiroll_constraint.hpp"
#include "edyn/constraints/doublewishbone_constraint.hpp"
#include "edyn/constraints/differential_constraint.hpp"
#include "edyn/constraints/tierod_constraint.hpp"
#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/constraints/tirecarcass_constraint.hpp"
#include "edyn/constraints/springdamper_constraint.hpp"
#include "edyn/constraints/spin_angle_constraint.hpp"
#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

inline constexpr size_t max_constraint_rows = 16;

struct constraint {
    std::array<entt::entity, 3> body {entt::null, entt::null, entt::null};
    std::variant<contact_constraint, 
                 point_constraint, 
                 distance_constraint,
                 soft_distance_constraint,
                 antiroll_constraint,
                 doublewishbone_constraint,
                 differential_constraint,
                 tierod_constraint,
                 contact_patch_constraint,
                 hinge_constraint,
                 generic_constraint,
                 tirecarcass_constraint,
                 springdamper_constraint,
                 spin_angle_constraint,
                 spin_constraint> var;
    std::array<entt::entity, max_constraint_rows> row = 
        make_array<max_constraint_rows>(entt::entity{entt::null});

    size_t num_rows() const {
        size_t count = 0;
        for (auto e : row) {
            if (e != entt::null) {
                ++count;
            }
        }
        return count;
    }
};

}

#endif // EDYN_COMP_CONSTRAINT_HPP