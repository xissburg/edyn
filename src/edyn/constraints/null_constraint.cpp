#include "edyn/constraints/null_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<null_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto con_view = registry.view<null_constraint>();

    con_view.each([&] (null_constraint &) {
        cache.con_num_rows.push_back(0);
    });
}

}
