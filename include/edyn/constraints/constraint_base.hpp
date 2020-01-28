#ifndef EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/dynamics/solver_stage.hpp"

namespace edyn {

struct constraint;
struct relation;

/**
 * @brief Base class of all constraints. Its sole purpose is to allow 
 * constraints to optionally define the update functions. It uses CRTP
 * to invoke the actual functions on different solver stages. It uses
 * SFINAE to optionally define overloads that would invoke the actual
 * functions on `Derived` (via `decltype` on the optional function). 
 * The supported functions are:
 * 
 * * @code{.cpp}
 *   void init(entt::entity, constraint &, relation &, entt::registry &)
 *   @endcode 
 * 
 *   Called once after a constraint is initially setup. This is usually the 
 *   place where `edyn::constraint_row`s are created, unless the number of 
 *   rows is dynamic.
 * 
 * * @code{.cpp}
 *   void prepare(entt::entity, constraint &, relation &, entt::registry &, scalar dt)
 *   @endcode
 * 
 *   Called before the solver iterations start. Here each `edyn::constraint_row`
 *   owned by the constraint should be setup based on current state of each 
 *   rigid body.
 * 
 * * @code{.cpp}
 *   void iteration(entt::entity, constraint &, relation &, entt::registry &, scalar dt)
 *   @endcode
 * 
 *   Called before each iteration. It gives the constraint the opportunity to
 *   adjust a `edyn::constraint_row` dynamically during iterations. One example
 *   is `edyn::contact_constraint` which adjusts the limits of the friction
 *   rows based on the normal impulse calculated so far.
 * 
 * Heavily based on how `entt::process` is setup. 
 * 
 * @tparam Derived Immediate subclass type (CRTP).
 */
template<typename Derived>
struct constraint_base {
    // Called once after a constraint is initially setup.
    template<typename Target = Derived>
    auto update(solver_stage_value_t<solver_stage::init>, 
                entt::entity ent,
                constraint &con, 
                const relation &rel, 
                entt::registry &reg, 
                scalar dt) -> decltype(std::declval<Target>().init(ent, con, rel, reg)) {
        static_cast<Target *>(this)->init(ent, con, rel, reg);
    }
    
    // Called before the solver iterations start.
    template<typename Target = Derived>
    auto update(solver_stage_value_t<solver_stage::prepare>, 
                entt::entity ent,
                constraint &con, 
                const relation &rel, 
                entt::registry &reg, 
                scalar dt) -> decltype(std::declval<Target>().prepare(ent, con, rel, reg, dt)) {
        static_cast<Target *>(this)->prepare(ent, con, rel, reg, dt);
    }

    // Called before solving each iteration.
    template<typename Target = Derived>
    auto update(solver_stage_value_t<solver_stage::iteration>, 
                entt::entity ent,
                constraint &con, 
                const relation &rel, 
                entt::registry &reg, 
                scalar dt) -> decltype(std::declval<Target>().iteration(ent, con, rel, reg, dt)) {
        static_cast<Target *>(this)->iteration(ent, con, rel, reg, dt);
    }

    // Base case which should be resolved to in case the specific function is 
    // not defined. It does nothing.
    template<solver_stage stage, typename... Args>
    void update(solver_stage_value_t<stage>, 
                entt::entity ent,
                constraint &con, 
                const relation &rel, 
                entt::registry &reg, 
                scalar dt) {
    }
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_BASE_HPP