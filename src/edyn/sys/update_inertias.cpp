#include "edyn/sys/update_inertias.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/tag.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_inertias(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>();
    view.each([] (orientation& orn, inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = basis * inv_I * transpose(basis);
    });
}

}
