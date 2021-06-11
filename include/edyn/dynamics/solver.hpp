#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <cstdint>
#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

class solver {
public:
    solver(entt::registry &);
    ~solver();

    void update(scalar dt);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
    row_cache m_row_cache;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
