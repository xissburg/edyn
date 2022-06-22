#ifndef EDYN_SYS_UPDATE_TIRE_STATE_HPP
#define EDYN_SYS_UPDATE_TIRE_STATE_HPP

#include <entt/entt.hpp>
#include "edyn/comp/origin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/tire_state.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

inline
void update_tire_state(entt::registry &registry, scalar dt) {
    auto ts_view = registry.view<graph_node, tire_state>();
    auto tr_view = registry.view<position, orientation>();
    auto vel_view = registry.view<linvel, angvel>();
    auto spin_view = registry.view<spin>();
    auto origin_view = registry.view<origin>();
    auto con_view = registry.view<contact_patch_constraint, contact_manifold>();
    auto &graph = registry.ctx().at<entity_graph>();

    ts_view.each([&] (auto entity, graph_node &node, tire_state &ts) {
        ts.other_entity = entt::null;

        auto &posA = tr_view.get<position>(entity);
        auto &ornA = tr_view.get<orientation>(entity);
        auto &linvelA = vel_view.get<linvel>(entity);
        auto &angvelA = vel_view.get<angvel>(entity);
        auto spinvelA = quaternion_x(ornA) * spin_view.get<spin>(entity).s;
        auto originA = origin_view.contains(entity) ? origin_view.get<origin>(entity) : static_cast<vector3>(posA);

        graph.visit_edges(node.node_index, [&] (auto edge_index) {
            auto edge_entity = graph.edge_entity(edge_index);

            if (!con_view.contains(edge_entity)) {
                return;
            }

            auto [contact_patch, manifold] = con_view.get(edge_entity);

            EDYN_ASSERT(registry.all_of<tire_state>(manifold.body[0]));

            ts.other_entity = manifold.body[1];
            ts.num_contacts = manifold.num_points;

            if (ts.num_contacts == 0) {
                return;
            }

            auto [posB, ornB] = tr_view.get(ts.other_entity);
            auto [linvelB, angvelB] = vel_view.get(ts.other_entity);
            auto spinvelB = vector3_zero;

            if (spin_view.contains(ts.other_entity)) {
                spinvelB = quaternion_x(ornB) * spin_view.get<spin>(ts.other_entity).s;
            }

            auto originB = origin_view.contains(ts.other_entity) ? origin_view.get<origin>(ts.other_entity) : static_cast<vector3>(posB);

            for (size_t i = 0; i < manifold.num_points; ++i) {
                auto &cp = manifold.get_point(i);
                auto &patch = contact_patch.patch[manifold.ids[i]];

                auto pivotA = to_world_space(cp.pivotA, originA, ornA);
                auto pivotB = to_world_space(cp.pivotB, originB, ornB);

                auto velA = linvelA + cross(angvelA + spinvelA, pivotA - posA);
                auto velB = linvelB + cross(angvelB + spinvelB, pivotB - posB);
                auto relvel = velA - velB;
                auto tan_relvel = project_direction(relvel, cp.normal);
                auto linvel_rel = project_direction(linvelA - velB, cp.normal);
                auto linspd_rel = length(linvel_rel);
                auto direction = linspd_rel > EDYN_EPSILON ? linvel_rel / linspd_rel : patch.lon_dir;

                auto &tire_cs = ts.contact_state[i];
                tire_cs.vertical_deflection = patch.deflection;
                tire_cs.friction_coefficient = cp.friction;
                tire_cs.sin_camber = patch.sin_camber;
                auto sin_slip_angle = std::clamp(dot(patch.lat_dir, direction), scalar(-1), scalar(1));
                tire_cs.slip_angle = std::asin(sin_slip_angle);
                auto vx = dot(linvel_rel, patch.lon_dir);
                auto vsx = dot(tan_relvel, patch.lon_dir);
                tire_cs.slip_ratio = -vsx / (to_sign(vx > 0) * std::max(std::abs(vx), scalar(0.001)));
                tire_cs.yaw_rate = dot(angvelA, cp.normal);
                tire_cs.slide_factor = patch.sliding_spd_avg;
                tire_cs.slide_ratio = patch.sliding_ratio;
                tire_cs.contact_patch_width = patch.width;
                tire_cs.contact_lifetime = cp.lifetime;
                tire_cs.lat_dir = patch.lat_dir;
                tire_cs.lon_dir = patch.lon_dir;
                tire_cs.normal = cp.normal;
                tire_cs.pivot = patch.pivot;
                tire_cs.position = patch.center;
                tire_cs.lin_vel = linvel_rel;

                // Add spring and damper impulses.
                tire_cs.Fz = cp.normal_impulse / dt; // Normal spring and damper.
                tire_cs.Fx = cp.friction_impulse[0] / dt; // Longitudinal spring.
                tire_cs.Fy = cp.friction_impulse[1] / dt; // Lateral spring.
                tire_cs.Mz = cp.spin_friction_impulse / dt; // Self-aliging spring.

                for (size_t i = 0; i < patch.tread_rows.size(); ++i) {
                    auto &tread_row = patch.tread_rows[i];
                    tire_cs.tread_rows[i].start_pos = tread_row.start_pos;
                    tire_cs.tread_rows[i].end_pos = tread_row.end_pos;

                    for (size_t j = 0; j < tread_row.bristles.size(); ++j) {
                        auto &bristle = tread_row.bristles[j];
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
