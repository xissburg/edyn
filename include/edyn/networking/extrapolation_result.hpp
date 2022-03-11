#ifndef EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP

#include <vector>
#include <memory>
#include <entt/entity/sparse_set.hpp>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/networking/extrapolation_component_pool.hpp"

namespace edyn {

struct extrapolation_result {
    std::vector<entt::entity> entities;
    std::vector<std::shared_ptr<extrapolation_component_pool>> pools;
    std::vector<contact_manifold> manifolds;
    bool terminated_early {false};
    double timestamp;

    void convert_locrem(entt::registry &registry, entity_map &emap) {
        for (auto &entity : entities) {
            entity = emap.locrem(entity);
        }

        for (auto &pool : pools) {
            pool->convert_locrem(registry, emap);
        }

        auto remove_it = std::remove_if(manifolds.begin(), manifolds.end(), [&] (contact_manifold &manifold) {
            if (!emap.has_rem(manifold.body[0]) || !emap.has_rem(manifold.body[1])) {
                return true;
            }

            manifold.body[0] = emap.locrem(manifold.body[0]);
            manifold.body[1] = emap.locrem(manifold.body[1]);
            return false;
        });
        manifolds.erase(remove_it, manifolds.end());
    }
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP
