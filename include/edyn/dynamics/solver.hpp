#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

class solver {
public:
    solver(entt::registry &);

    void update(scalar dt);

    void on_construct_constraint_row(entt::registry &, entt::entity);
    void on_destroy_constraint_row(entt::registry &, entt::entity);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
    bool m_constraints_changed;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP