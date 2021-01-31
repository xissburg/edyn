#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

struct job;
struct solver_context;

class solver {
    struct state {
        size_t iteration {0};
        scalar dt;
    };

    void init_new_rows();
    void partite_constraint_graph();
    void run_async_iteration();
    void dispatch_solver_job();

public:
    solver(entt::registry &);
    ~solver();

    bool parallelizable() const;
    void update(scalar dt);
    void start_async_update(scalar dt, const job &completion);
    bool continue_async_update();
    void finish_async_update();

    void on_change_constraint_graph(entt::registry &, entt::entity);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
    bool m_constraints_changed;
    state m_state;
    std::vector<entt::entity> m_new_rows;
    std::unique_ptr<solver_context> m_context;
    size_t m_num_constraint_groups;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP