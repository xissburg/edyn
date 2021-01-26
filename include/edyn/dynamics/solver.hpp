#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

struct job;

class solver {
    struct state {
        size_t color {0};
        size_t edge_index {0};
        size_t iteration {0};
        scalar dt;
    };

public:
    solver(entt::registry &);

    bool parallelizable() const;
    void update(scalar dt);
    void start_async_update(scalar dt);
    bool continue_async_update(const job &completion);
    void finish_async_update();

    void on_construct_constraint_row(entt::registry &, entt::entity);
    void on_destroy_constraint_row(entt::registry &, entt::entity);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
    bool m_constraints_changed;
    state m_state;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP