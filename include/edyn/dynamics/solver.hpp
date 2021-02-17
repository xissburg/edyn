#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <cstdint>
#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

class solver {
public:
    solver(entt::registry &);
    ~solver();

    void update(scalar dt);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP