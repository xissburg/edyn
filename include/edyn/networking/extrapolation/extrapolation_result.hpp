#ifndef EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_RESULT_HPP

#include <entt/entity/fwd.hpp>
#include <vector>
#include <memory>
#include <entt/entity/sparse_set.hpp>
#include "edyn/collision/contact_point.hpp"
#include "edyn/replication/registry_operation.hpp"

namespace edyn {

struct extrapolation_result {
    struct contact_manifold_info {
        std::array<entt::entity, 2> body;
        entt::entity contact_entity;
    };

    struct contact_point_info {
        contact_point pt;
        contact_point_list list;
        contact_point_geometry geom;
        contact_point_impulse imp;
    };

    registry_operation ops;
    std::vector<contact_manifold_info> manifolds;
    entt::storage<contact_point_info> contacts;

    bool terminated_early {false};
    double timestamp;

    void remap(entity_map &emap) {
        ops.remap(emap);

        auto remove_it = std::remove_if(manifolds.begin(), manifolds.end(), [&](contact_manifold_info &manifold) {
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
