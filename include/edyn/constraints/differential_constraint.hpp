#ifndef EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct differential_constraint : public constraint_base<differential_constraint> {
    scalar ratio;

    void init(constraint &, const relation &, entt::registry &);
    void prepare(constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP