#ifndef EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP

#include <vector>
#include <memory>
#include <entt/entity/sparse_set.hpp>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/replication/registry_operation.hpp"

namespace edyn {

struct extrapolation_result {
    registry_operation ops;
    std::vector<contact_manifold> manifolds;
    bool terminated_early {false};
    double timestamp;

    void remap(entity_map &emap) {
        ops.remap(emap);

        auto remove_it = std::remove_if(manifolds.begin(), manifolds.end(), [&](contact_manifold &manifold) {
            if (!emap.contains(manifold.body[0]) || !emap.contains(manifold.body[1])) {
                return true;
            }

            manifold.body[0] = emap.at(manifold.body[0]);
            manifold.body[1] = emap.at(manifold.body[1]);
            return false;
        });
        manifolds.erase(remove_it, manifolds.end());
    }
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP
