#ifndef EDYN_SYS_UPDATE_TIRE_STATS_HPP
#define EDYN_SYS_UPDATE_TIRE_STATS_HPP

#include <entt/entt.hpp>
#include "edyn/config/constants.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/tire_stats.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

inline
void update_tire_stats(entt::registry &registry, scalar dt) {
    auto stats_view = registry.view<graph_node, tire_stats>();
    auto tr_view = registry.view<position, orientation>();
    auto vel_view = registry.view<linvel, angvel>();
    auto spin_view = registry.view<spin>();
    auto patch_view = registry.view<contact_patch_constraint>();
    auto &graph = registry.ctx().at<entity_graph>();

    for (auto [entity, node, stats] : stats_view.each()) {
        auto &posA = tr_view.get<position>(entity);
        auto &ornA = tr_view.get<orientation>(entity);
        auto &linvelA = vel_view.get<linvel>(entity);
        auto &angvelA = vel_view.get<angvel>(entity);
        auto spinvelA = quaternion_x(ornA) * spin_view.get<spin>(entity).s;
        // Must create another reference to circumvent the compiler limitation:
        // "Reference to local binding declared in enclosing function".
        auto &ts = stats;
        ts.num_contacts = 0;

        graph.visit_edges(node.node_index, [&](auto edge_index) {
            auto edge_entity = graph.edge_entity(edge_index);

            if (!patch_view.contains(edge_entity)) {
                return;
            }

            auto [con] = patch_view.get(edge_entity);

            if (con.num_patches == 0) {
                return;
            }

            EDYN_ASSERT(registry.all_of<tire_stats>(con.body[0]));


            auto other_entity = con.body[1];
            auto [posB, ornB] = tr_view.get(other_entity);
            auto [linvelB, angvelB] = vel_view.get(other_entity);
            auto spinvelB = vector3_zero;

            if (spin_view.contains(other_entity)) {
                spinvelB = quaternion_x(ornB) * spin_view.get<spin>(other_entity).s;
            }

            for (size_t i = 0; i < con.num_patches; ++i) {
                if (ts.num_contacts == max_contacts) {
                    break;
                }

                auto &patch = con.patches[i];

                auto velA = linvelA + cross(angvelA + spinvelA, patch.pivot - posA);
                auto velB = linvelB + cross(angvelB + spinvelB, patch.pivot - posB);
                auto relvel = velA - velB;
                auto tan_relvel = project_direction(relvel, patch.normal);
                auto linvel_rel = project_direction(linvelA - velB, patch.normal);
                auto linspd_rel = length(linvel_rel);
                auto direction = linspd_rel > EDYN_EPSILON ? linvel_rel / linspd_rel : patch.lon_dir;

                auto &tire_cs = ts.contact_stats[ts.num_contacts++];
                tire_cs.other_entity = other_entity;
                tire_cs.patch_entity = edge_entity;
                tire_cs.vertical_deflection = patch.deflection;
                tire_cs.friction_coefficient = patch.friction;
                tire_cs.sin_camber = patch.sin_camber;
                auto sin_slip_angle = std::clamp(dot(patch.lat_dir, direction), scalar(-1), scalar(1));
                tire_cs.slip_angle = std::asin(sin_slip_angle);
                auto vx = dot(linvel_rel, patch.lon_dir);
                auto vsx = dot(tan_relvel, patch.lon_dir);
                tire_cs.slip_ratio = -vsx / (to_sign(vx > 0) * std::max(std::abs(vx), scalar(0.001)));
                tire_cs.yaw_rate = dot(angvelA, patch.normal);
                tire_cs.slide_speed = patch.sliding_spd_avg;
                tire_cs.slide_ratio = patch.sliding_ratio;
                tire_cs.contact_patch_width = patch.width;
                tire_cs.contact_lifetime = patch.lifetime;
                tire_cs.lat_dir = patch.lat_dir;
                tire_cs.lon_dir = patch.lon_dir;
                tire_cs.normal = patch.normal;
                tire_cs.pivot = patch.pivot;
                tire_cs.position = patch.center;
                tire_cs.lin_vel = linvel_rel;

                const auto &impulse = patch.applied_impulse;
                tire_cs.Fz = impulse.normal / dt;
                tire_cs.Fx = impulse.longitudinal / dt;
                tire_cs.Fy = impulse.lateral / dt;
                tire_cs.Mz = impulse.aligning / dt;

                for (size_t i = 0; i < patch.tread_rows.size(); ++i) {
                    auto &tread_row = patch.tread_rows[i];
                    tire_cs.tread_rows[i].start_pos = tread_row.start_pos;
                    tire_cs.tread_rows[i].end_pos = tread_row.end_pos;

                    for (size_t j = 0; j < tread_row.bristles.size(); ++j) {
                        auto &bristle = tread_row.bristles[j];
                        tire_cs.tread_rows[i].bristles[j] = tire_bristle_stats{
                            bristle.root, bristle.tip,
                            bristle.friction, bristle.sliding_spd
                        };
                    }
                }
            }
        });
    }
}

}

#endif // EDYN_SYS_UPDATE_TIRE_STATS_HPP
