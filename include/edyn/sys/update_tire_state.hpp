#ifndef EDYN_SYS_UPDATE_TIRE_STATE_HPP
#define EDYN_SYS_UPDATE_TIRE_STATE_HPP

#include <entt/entt.hpp>
#include "edyn/comp/tire_state.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

inline
void update_tire_state(entt::registry &registry, scalar dt) {
    auto view = registry.view<const relation_container, tire_state>();
    view.each([&] (auto ent, const relation_container &rels, tire_state &ts) {
        ts.in_contact = false;
        ts.other_entity = entt::null;

        const auto &posA = registry.get<position>(ent);
        const auto &ornA = registry.get<orientation>(ent);

        const auto &linvelA = registry.get<linvel>(ent);
        const auto &angvelA = registry.get<angvel>(ent);
        
        const auto &spinA = registry.get<spin>(ent);
        const auto spin_angvelA = angvelA + quaternion_x(ornA) * spinA;

        for (auto rel_ent : rels.entities) {
            auto con = registry.try_get<constraint>(rel_ent);
            if (!con) continue;

            auto contact_patch = std::get_if<contact_patch_constraint>(&con->var);
            if (!contact_patch) continue;

            auto &manifold = registry.get<contact_manifold>(rel_ent);
            ts.in_contact = manifold.num_points > 0;
            if (!ts.in_contact) continue;

            auto &rel = registry.get<relation>(rel_ent);
            ts.other_entity = rel.entity[1];

            const auto &posB = registry.get<position>(rel.entity[1]);
            const auto &linvelB = registry.get<linvel>(rel.entity[1]);
            const auto &angvelB = registry.get<angvel>(rel.entity[1]);

            auto velA = linvelA + cross(spin_angvelA, contact_patch->m_pivot - posA);
            auto velB = linvelB + cross(angvelB, contact_patch->m_pivot - posB);
            auto relvel = velA - velB;
            auto tan_relvel = project_direction(relvel, contact_patch->m_normal);
            auto linvel_rel = project_direction(linvelA - linvelB, contact_patch->m_normal);
            auto linspd_rel = length(linvel_rel);
            auto direction = linspd_rel > EDYN_EPSILON ? linvel_rel / linspd_rel : contact_patch->m_lon_dir;
            auto cp_ent = manifold.point_entity[contact_patch->m_manifold_point_index];
            auto &cp = registry.get<contact_point>(cp_ent);

            ts.vertical_deflection = contact_patch->m_deflection;
            ts.speed = linspd_rel;
            ts.friction_coefficient = contact_patch->m_friction_coefficient;
            ts.sin_camber = contact_patch->m_sin_camber;
            ts.slip_angle = std::asin(dot(contact_patch->m_lat_dir, direction));
            auto vx = dot(linvel_rel, contact_patch->m_lon_dir);
            auto vsx = dot(tan_relvel, contact_patch->m_lon_dir);
            ts.slip_ratio = std::abs(vx) > 0.001 ? -vsx/vx : -vsx;
            ts.yaw_rate = dot(angvelA, contact_patch->m_normal);
            ts.slide_factor = contact_patch->m_sliding_spd_avg;
            ts.contact_patch_length = contact_patch->m_contact_len_avg;
            ts.contact_patch_width = contact_patch->m_contact_width;
            ts.contact_lifetime = cp.lifetime;
            ts.lat_dir = contact_patch->m_lat_dir;
            ts.lon_dir = contact_patch->m_lon_dir;
            ts.normal = contact_patch->m_normal;
            ts.position = contact_patch->m_pivot;
            ts.lin_vel = linvel_rel;

            if (con->num_rows == 0) continue;

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