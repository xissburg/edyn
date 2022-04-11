#include "edyn/networking/sys/update_network_dirty.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/networking/comp/network_dirty.hpp"
#include "edyn/networking/networking_external.hpp"
#include <entt/entity/entity.hpp>

namespace edyn {

void update_network_dirty(entt::registry &registry, double time) {
    // TODO: expiry duration should be proportional to packet loss.
    double expiry_duration = 0.4;

    // Remove networked component types that changed a while ago, as they are
    // assumed to have reached the other end by now.
    for (auto [entity, n_dirty] : registry.view<network_dirty>().each()) {
        n_dirty.expire(time, expiry_duration);

        if (n_dirty.empty()) {
            registry.erase<network_dirty>(entity);
        }
    }

    // Insert components marked as dirty.
    for (auto [entity, dirty] : registry.view<dirty, networked_tag>().each()) {
        for (auto id : dirty.updated_ids) {
            if ((*g_is_networked_component)(id)) {
                auto &n_dirty = registry.get_or_emplace<network_dirty>(entity);
                n_dirty.insert(id, time);
            }
        }
    }
}

}
