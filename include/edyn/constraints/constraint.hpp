#ifndef EDYN_CONSTRAINTS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_HPP

#include <tuple>
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
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

using constrained_entities = std::array<entt::entity, max_constrained_entities>;

using constraints_tuple_t = std::tuple<
    point_constraint, 
    distance_constraint,
    soft_distance_constraint,
    hinge_constraint,
    generic_constraint,
    contact_constraint 
>;

struct constraint_impulse {
    std::array<scalar, max_constraint_rows> values;
};

// Preparation

template<typename C>
void prepare_constraints(entt::registry &, row_cache &, scalar dt);

template<> inline
void prepare_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_contact_constraints(registry, cache, dt);
}

template<> inline
void prepare_constraints<distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_distance_constraints(registry, cache, dt);
}

template<> inline
void prepare_constraints<generic_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_generic_constraints(registry, cache, dt);
}

template<> inline
void prepare_constraints<hinge_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_hinge_constraints(registry, cache, dt);
}

template<> inline
void prepare_constraints<point_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_point_constraints(registry, cache, dt);
}

template<> inline
void prepare_constraints<soft_distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    prepare_soft_distance_constraints(registry, cache, dt);
}

inline
void prepare_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    std::apply([&] (auto ... c) {
        (prepare_constraints<decltype(c)>(registry, cache, dt), ...);
    }, constraints_tuple_t{});
}

// Iteration

template<typename C>
void iterate_constraints(entt::registry &, row_cache &, scalar dt);

template<> inline
void iterate_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    iterate_contact_constraints(registry, cache, dt);
}

template<> inline
void iterate_constraints<distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    //iterate_distance_constraints(registry, cache, dt);
}

template<> inline
void iterate_constraints<generic_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    //iterate_generic_constraints(registry, cache, dt);
}

template<> inline
void iterate_constraints<hinge_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    //iterate_hinge_constraints(registry, cache, dt);
}

template<> inline
void iterate_constraints<point_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    //iterate_point_constraints(registry, cache, dt);
}

template<> inline
void iterate_constraints<soft_distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    iterate_soft_distance_constraints(registry, cache, dt);
}

inline
void iterate_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    std::apply([&] (auto ... c) {
        (iterate_constraints<decltype(c)>(registry, cache, dt), ...);
    }, constraints_tuple_t{});
}

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_HPP