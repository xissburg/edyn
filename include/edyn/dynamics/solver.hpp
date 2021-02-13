#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/util/entity_set.hpp"

namespace edyn {

struct job;
struct solver_context;
struct island_node;
struct constraint;

class solver {
    void sort_rows();

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