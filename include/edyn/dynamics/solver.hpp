#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <memory>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/parallel/atomic_counter.hpp"

namespace edyn {

struct job;
struct constraint_row;

scalar solve(constraint_row &row);

class solver final {

public:
    solver(entt::registry &);

    void update(bool mt);

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    std::unique_ptr<atomic_counter> m_counter;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
