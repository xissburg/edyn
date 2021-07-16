#ifndef EDYN_SYS_UPDATE_TIRE_STATE_HPP
#define EDYN_SYS_UPDATE_TIRE_STATE_HPP

#include <entt/entt.hpp>
#include "edyn/comp/tire_state.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/constraints/constraint_impulse.hpp"

namespace edyn {

inline
void update_tire_state(entt::registry &registry, scalar dt) {
    auto ts_view = registry.view<graph_node, tire_state>();
    auto tr_view = registry.view<position, orientation>();
    auto vel_view = registry.view<linvel, angvel>();
    auto &graph = registry.ctx<entity_graph>();

    ts_view.each([&] (auto entity, graph_node &node, tire_state &ts) {
        ts.other_entity = entt::null;

        auto &ornA = tr_view.get<orientation>(entity);
        auto [linvelA, angvelA] = vel_view.get<linvel, angvel>(entity);

        const auto &spinA = registry.get<spin>(entity);
        const auto spin_angvelA = angvelA + quaternion_x(ornA) * spinA;

        graph.visit_edges(node.node_index, [&] (auto edge_entity) {
            auto *manifold = registry.try_get<contact_manifold>(edge_entity);

            if (!manifold) {
                return;
            }

            ts.other_entity = manifold->body[1];
            ts.num_contacts = manifold->num_points();

            if (ts.num_contacts == 0) {
                return;
            }

            auto [posB, ornB] = tr_view.get<position, orientation>(manifold->body[1]);
            auto [linvelB, angvelB] = vel_view.get<linvel, angvel>(manifold->body[1]);

            for (size_t i = 0; i < manifold->num_points(); ++i) {
                auto &cp = registry.get<contact_point>(manifold->point[i]);
                auto &contact_patch = registry.get<contact_patch_constraint>(manifold->point[i]);

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
                tire_cs.pivot = contact_patch.m_pivot;
                tire_cs.position = contact_patch.m_center;
                tire_cs.lin_vel = linvel_rel;

                auto &imp = registry.get<constraint_impulse>(manifold->point[i]);
                // Add spring and damper impulses.
                tire_cs.Fz = (imp.values[0] + imp.values[1]) / dt;
                tire_cs.Fx = (imp.values[2] + imp.values[3]) / dt;
                tire_cs.Fy = (imp.values[4] + imp.values[5]) / dt;
                tire_cs.Mz = (imp.values[6] + imp.values[7]) / dt;

                for (size_t i = 0; i < contact_patch.m_tread_rows.size(); ++i) {
                    tire_cs.tread_rows[i].start_pos = to_world_space(contact_patch.m_tread_rows[i].start_posB, posB, ornB);
                    tire_cs.tread_rows[i].end_pos = to_world_space(contact_patch.m_tread_rows[i].end_posB, posB, ornB);

                    for (size_t j = 0; j < contact_patch.m_tread_rows[i].bristles.size(); ++j) {
                        auto &bristle = contact_patch.m_tread_rows[i].bristles[j];
                        tire_cs.tread_rows[i].bristles[j] = tire_bristle_state{
                            bristle.root, bristle.tip,
                            bristle.friction, bristle.sliding_spd
                        };
                    }
                }
            }
        });
    });
}

}

#endif // EDYN_SYS_UPDATE_TIRE_STATE_HPP