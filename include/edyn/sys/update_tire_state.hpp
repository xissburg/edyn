#ifndef EDYN_SYS_UPDATE_TIRE_STATE_HPP
#define EDYN_SYS_UPDATE_TIRE_STATE_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/center_of_mass.hpp"
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
    auto spin_view = registry.view<spin>();
    auto com_view = registry.view<center_of_mass>();
    auto &graph = registry.ctx<entity_graph>();

    ts_view.each([&] (auto entity, graph_node &node, tire_state &ts) {
        ts.other_entity = entt::null;

        auto &posA = tr_view.get<position>(entity);
        auto &ornA = tr_view.get<orientation>(entity);
        auto &linvelA = vel_view.get<linvel>(entity);
        auto &angvelA = vel_view.get<angvel>(entity);
        auto spinvelA = quaternion_x(ornA) * spin_view.get(entity).s;

        auto originA = static_cast<vector3>(posA);

        if (com_view.contains(entity)) {
            auto &com = com_view.get(entity);
            originA = to_world_space(-com, posA, ornA);
        }

        graph.visit_edges(node.node_index, [&] (auto edge_entity) {
            auto *manifold = registry.try_get<contact_manifold>(edge_entity);

            if (!manifold) {
                return;
            }

            EDYN_ASSERT(manifold->body[0] == entity);
            ts.other_entity = manifold->body[1];
            ts.num_contacts = manifold->num_points();

            if (ts.num_contacts == 0) {
                return;
            }

            auto [posB, ornB] = tr_view.get<position, orientation>(ts.other_entity);
            auto [linvelB, angvelB] = vel_view.get<linvel, angvel>(ts.other_entity);
            auto spinvelB = vector3_zero;

            if (spin_view.contains(ts.other_entity)) {
                spinvelB = quaternion_x(ornB) * spin_view.get(ts.other_entity).s;
            }

            auto originB = static_cast<vector3>(posB);

            if (com_view.contains(ts.other_entity)) {
                auto &com = com_view.get(ts.other_entity);
                originB = to_world_space(-com, posB, ornB);
            }

            for (size_t i = 0; i < manifold->num_points(); ++i) {
                auto point_entity = manifold->point[i];

                if (!registry.has<contact_patch_constraint>(point_entity)) {
                    continue;
                }

                auto &contact_patch = registry.get<contact_patch_constraint>(point_entity);
                auto &cp = registry.get<contact_point>(point_entity);

                auto pivotA = to_world_space(cp.pivotA, originA, ornA);
                auto pivotB = to_world_space(cp.pivotB, originB, ornB);

                auto velA = linvelA + cross(angvelA + spinvelA, pivotA - posA);
                auto velB = linvelB + cross(angvelB + spinvelB, pivotB - posB);
                auto relvel = velA - velB;
                auto tan_relvel = project_direction(relvel, cp.normal);
                auto linvel_rel = project_direction(linvelA - velB, cp.normal);
                auto linspd_rel = length(linvel_rel);
                auto direction = linspd_rel > EDYN_EPSILON ? linvel_rel / linspd_rel : contact_patch.m_lon_dir;

                auto &tire_cs = ts.contact_state[i];
                tire_cs.vertical_deflection = contact_patch.m_deflection;
                tire_cs.friction_coefficient = cp.friction;
                tire_cs.sin_camber = contact_patch.m_sin_camber;
                auto sin_slip_angle = std::clamp(dot(contact_patch.m_lat_dir, direction), scalar(-1), scalar(1));
                tire_cs.slip_angle = std::asin(sin_slip_angle);
                auto vx = dot(linvel_rel, contact_patch.m_lon_dir);
                auto vsx = dot(tan_relvel, contact_patch.m_lon_dir);
                tire_cs.slip_ratio = -vsx / (to_sign(vx > 0) * std::max(std::abs(vx), scalar(0.001)));
                tire_cs.yaw_rate = dot(angvelA, cp.normal);
                tire_cs.slide_factor = contact_patch.m_sliding_spd_avg;
                tire_cs.slide_ratio = contact_patch.m_sliding_ratio;
                tire_cs.contact_patch_width = contact_patch.m_contact_width;
                tire_cs.contact_lifetime = cp.lifetime;
                tire_cs.lat_dir = contact_patch.m_lat_dir;
                tire_cs.lon_dir = contact_patch.m_lon_dir;
                tire_cs.normal = cp.normal;
                tire_cs.pivot = contact_patch.m_pivot;
                tire_cs.position = contact_patch.m_center;
                tire_cs.lin_vel = linvel_rel;

                auto &imp = registry.get<constraint_impulse>(point_entity);
                // Add spring and damper impulses.
                tire_cs.Fz = (imp.values[0] + imp.values[1]) / dt; // Normal spring and damper.
                tire_cs.Fx = imp.values[2] / dt; // Longitudinal spring.
                tire_cs.Fy = imp.values[3] / dt; // Lateral spring.
                tire_cs.Mz = imp.values[4] / dt; // Self-aliging spring.

                for (size_t i = 0; i < contact_patch.m_tread_rows.size(); ++i) {
                    tire_cs.tread_rows[i].start_pos = contact_patch.m_tread_rows[i].start_pos;
                    tire_cs.tread_rows[i].end_pos = contact_patch.m_tread_rows[i].end_pos;

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
