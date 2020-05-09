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

scalar get_effective_mass(const constraint_row &row, 
                          const mass_inv &inv_mA, const inertia_world_inv &inv_IA,
                          const mass_inv &inv_mB, const inertia_world_inv &inv_IB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * inv_mA +
                     dot(inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * inv_mB +
                     dot(inv_IB * row.J[3], row.J[3]);
    auto eff_mass = scalar(1) / J_invM_JT;
    return eff_mass;
}

}