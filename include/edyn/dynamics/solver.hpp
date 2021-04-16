#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <cstdint>
#include <entt/fwd.hpp>
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

void prepare_row(const constraint_row &row, constraint_row_data &data,
                 const vector3 &linvelA, const vector3 &linvelB,
                 const vector3 &angvelA, const vector3 &angvelB);
void warm_start(constraint_row_data &data);

}

#endif // EDYN_DYNAMICS_SOLVER_HPP