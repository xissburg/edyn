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
    auto ts_view = registry.view<const relation_container, tire_state>();
    auto orn_view = registry.view< orientation>();
    auto vel_view = registry.view<linvel, angvel>();
    auto con_row_view = registry.view<constraint_row>();

    ts_view.each([&] (auto entity, const relation_container &rels, tire_state &ts) {
        ts.other_entity = entt::null;

        auto &ornA = orn_view.get(entity);
        auto [linvelA, angvelA] = vel_view.get<linvel, angvel>(entity);
        
        const auto &spinA = registry.get<spin>(entity);
        const auto spin_angvelA = angvelA + quaternion_x(ornA) * spinA;

        for (auto rel_ent : rels.entities) {
            auto *manifold = registry.try_get<contact_manifold>(rel_ent);
            if (!manifold) {
                continue;
            }

            auto manifold_rel = registry.get<relation>(rel_ent);
            ts.other_entity = manifold_rel.entity[1];
            ts.num_contacts = manifold->num_points;
            
            if (ts.num_contacts == 0) continue;

            auto ornB = orn_view.get(manifold_rel.entity[1]);
            auto [linvelB, angvelB] = vel_view.get<linvel, angvel>(manifold_rel.entity[1]);

            for (size_t i = 0; i < manifold->num_points; ++i) {
                auto [con, cp] = registry.get<constraint, contact_point>(manifold->point_entity[i]);
                auto &contact_patch = std::get<contact_patch_constraint>(con.var);

                auto velA = linvelA + cross(spin_angvelA, rotate(ornA, cp.pivotA));
                auto velB = linvelB + cross(angvelB, rotate(ornB, cp.pivotB));
                auto relvel = velA - velB;
                auto tan_relvel = project_direction(relvel, contact_patch.m_normal);
                auto linvel_rel = project_direction(linvelA - linvelB, contact_patch.m_normal);
                auto linspd_rel = length(linvel_rel);
                auto direction = linspd_rel > EDYN_EPSILON ? linvel_rel / linspd_rel : contact_patch.m_lon_dir;

                auto &tire_cs = ts.contact_state[i];
                tire_cs.vertical_deflection = contact_patch.m_deflection;
                tire_cs.speed = linspd_rel;
                tire_cs.friction_coefficient = cp.friction;
                tire_cs.sin_camber = contact_patch.m_sin_camber;
                auto sin_slip_angle = std::clamp(dot(contact_patch.m_lat_dir, direction), scalar(-1), scalar(1));
                tire_cs.slip_angle = std::asin(sin_slip_angle);
                auto vx = dot(linvel_rel, contact_patch.m_lon_dir);
                auto vsx = dot(tan_relvel, contact_patch.m_lon_dir);
                tire_cs.slip_ratio = std::abs(vx) > 0.001 ? -vsx/vx : -vsx;
                tire_cs.yaw_rate = dot(angvelA, contact_patch.m_normal);
                tire_cs.slide_factor = contact_patch.m_sliding_spd_avg;
                tire_cs.contact_patch_length = contact_patch.m_contact_len_avg;
                tire_cs.contact_patch_width = contact_patch.m_contact_width;
                tire_cs.contact_lifetime = cp.lifetime;
                tire_cs.lat_dir = contact_patch.m_lat_dir;
                tire_cs.lon_dir = contact_patch.m_lon_dir;
                tire_cs.normal = contact_patch.m_normal;
                tire_cs.position = contact_patch.m_pivot;
                tire_cs.lin_vel = linvel_rel;

                auto &normal_row = con_row_view.get(con.row[0]);
                auto &lon_row = con_row_view.get(con.row[1]);
                auto &lat_row = con_row_view.get(con.row[2]);
                auto &align_row = con_row_view.get(con.row[3]);

                tire_cs.Fz = normal_row.impulse / dt;
                tire_cs.Fx = lon_row.impulse / dt;
                tire_cs.Fy = lat_row.impulse / dt;
                tire_cs.Mz = align_row.impulse / dt;
            }

            // Only process one manifold. The tire state is a component of the
            // tire, not the relation.
            break;
        }
    });
}

}

#endif // EDYN_SYS_UPDATE_TIRE_STATE_HPP