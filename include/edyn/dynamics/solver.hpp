#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <entt/entt.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

class solver {
public:
    solver(entt::registry &);

    void update(scalar dt);

    uint32_t iterations {10};

private:
    entt::registry *registry;
    std::vector<entt::scoped_connection> connections;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP