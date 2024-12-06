#include "edyn/networking/util/import_contact_manifolds.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/replication/entity_map.hpp"

namespace edyn {

void replace_manifold(entt::registry &registry, contact_manifold &manifold,
                      const contact_manifold_map &manifold_map) {
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(manifold.body[0]));
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(manifold.body[1]));
    entt::entity manifold_entity;

    // Find a matching manifold and replace it...
    if (manifold_map.contains(manifold.body[0], manifold.body[1])) {
        manifold_entity = manifold_map.get(manifold.body[0], manifold.body[1]);
    } else {
        // ...or create a new one and assign a new value to it.
        // Important remark: `make_contact_manifold` does not necessarily
        // create the `contact_manifold` with the bodies in the same order
        // that's passed in the arguments.
        auto separation_threshold = contact_breaking_threshold * scalar(1.3);
        manifold_entity = make_contact_manifold(registry,
                                                manifold.body[0], manifold.body[1],
                                                separation_threshold);
    }

    auto &original_manifold = registry.get<contact_manifold>(manifold_entity);

    // Must maintain bodies in the same order.
    if (manifold.body[0] != original_manifold.body[0]) {
        EDYN_ASSERT(manifold.body[1] == original_manifold.body[0]);
        EDYN_ASSERT(manifold.body[0] == original_manifold.body[1]);
        swap_manifold(manifold);
    }

    registry.replace<contact_manifold>(manifold_entity, manifold);
}

void import_contact_manifolds(entt::registry &registry, const entity_map &emap,
                              const std::vector<contact_manifold> &manifolds) {
    const auto &manifold_map = registry.ctx().get<contact_manifold_map>();

    for (auto manifold : manifolds) {
        if (!emap.contains(manifold.body[0]) || !emap.contains(manifold.body[1])) {
            continue;
        }

        manifold.body[0] = emap.at(manifold.body[0]);
        manifold.body[1] = emap.at(manifold.body[1]);

        if (!registry.valid(manifold.body[0]) || !registry.valid(manifold.body[1])) {
            continue;
        }

        replace_manifold(registry, manifold, manifold_map);
    }
}

void import_contact_manifolds(entt::registry &registry,
                              const std::vector<contact_manifold> &manifolds) {
    const auto &manifold_map = registry.ctx().get<contact_manifold_map>();

    for (auto manifold : manifolds) {
        if (!registry.valid(manifold.body[0]) || !registry.valid(manifold.body[1])) {
            continue;
        }

        replace_manifold(registry, manifold, manifold_map);
    }
}

}
