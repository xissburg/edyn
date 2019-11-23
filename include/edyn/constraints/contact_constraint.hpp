#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint;
struct relation;

struct contact_constraint {
    static constexpr size_t num_rows = 4;

    void prepare(constraint *, const relation *, entt::registry &, scalar dt);
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP