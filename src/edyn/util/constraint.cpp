#include "edyn/util/constraint.hpp"

namespace edyn {

void set_constraint_enabled(entt::entity entity, entt::registry &registry, bool enabled) {
    auto& con = registry.get<constraint>(entity);
    
    if (enabled) {
        registry.reset<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.reset<disabled_tag>(con.row[i]);
        }
    } else {
        registry.assign_or_replace<disabled_tag>(entity);

        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.assign_or_replace<disabled_tag>(con.row[i]);
        }
    }
}

}