#ifndef EDYN_SYS_UPDATE_TIRE_STATE_HPP
#define EDYN_SYS_UPDATE_TIRE_STATE_HPP

#include <entt/entt.hpp>
#include "edyn/comp/tire_state.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"

namespace edyn {

inline
void update_tire_state(entt::registry &registry, scalar dt) {
    auto view = registry.view<const relation_container, tire_state>();
    view.each([&] (auto ent, const relation_container &rels, tire_state &ts) {
        ts.in_contact = false;
        ts.other_entity = entt::null;

        for (auto rel_ent : rels.entities) {
            auto con = registry.try_get<constraint>(rel_ent);
            if (!con) continue;
            auto contact_patch = std::get_if<contact_patch_constraint>(&con->var);
            if (!contact_patch) continue;
                
            auto &rel = registry.get<relation>(rel_ent);
            ts.other_entity = rel.entity[1];
            ts.in_contact = true;

            auto &normal_row = registry.get<constraint_row>(con->row[0]);
            auto &lon_row = registry.get<constraint_row>(con->row[1]);
            auto &lat_row = registry.get<constraint_row>(con->row[2]);
            auto &align_row = registry.get<constraint_row>(con->row[3]);
            
            ts.Fz = normal_row.impulse / dt;
            ts.Fx = lon_row.impulse / dt;
            ts.Fy = lat_row.impulse / dt;
            ts.Mz = align_row.impulse / dt;

            break;
        }
    });
}

}

#endif // EDYN_SYS_UPDATE_TIRE_STATE_HPP