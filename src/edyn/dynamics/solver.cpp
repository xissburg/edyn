#include "edyn/dynamics/solver.hpp"
#include "edyn/comp/constraint.hpp"

namespace edyn {

void solver::update(scalar dt) {
    registry->view<constraint>().each([] (auto, auto &con) {
        std::visit([&] (auto&& c) {
            c.prepare(&con, *registry, dt);
        }, con.var);
    });

    for (size_t i = 0; i < iterations; ++i) {
        registry->view<constraint_row>().each([&] (auto, auto &row) {
            solve(row);
        });
    }
}

void solver::solve(constraint_row &row) {
    
}

}