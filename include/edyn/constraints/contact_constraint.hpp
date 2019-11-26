#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

struct constraint;
struct relation;

struct contact_constraint {
    contact_manifold manifold {};
    scalar prev_dt;

    void init(constraint *, const relation *, entt::registry &);
    void prepare(constraint *, const relation *, entt::registry &, scalar dt);
    void before_solve(constraint *, const relation *, entt::registry &, scalar dt);
    void finish(constraint *, const relation *, entt::registry &);
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP