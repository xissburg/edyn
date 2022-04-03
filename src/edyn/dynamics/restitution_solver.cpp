#include "edyn/dynamics/restitution_solver.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename BodyView, typename OriginView>
scalar get_manifold_min_relvel(const contact_manifold &manifold, const BodyView &body_view,
                               const OriginView &origin_view) {
    if (manifold.num_points == 0) {
        return EDYN_SCALAR_MAX;
    }

    auto [posA, ornA, linvelA, angvelA] =
        body_view.template get<position, orientation, linvel, angvel>(manifold.body[0]);
    auto [posB, ornB, linvelB, angvelB] =
        body_view.template get<position, orientation, linvel, angvel>(manifold.body[1]);

    auto originA = origin_view.contains(manifold.body[0]) ? origin_view.template get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(manifold.body[1]) ? origin_view.template get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

    auto min_relvel = EDYN_SCALAR_MAX;

    for (size_t pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);
        auto normal = cp.normal;
        auto pivotA = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB = to_world_space(cp.pivotB, originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        auto vA = linvelA + cross(angvelA, rA);
        auto vB = linvelB + cross(angvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);
        min_relvel = std::min(normal_relvel, min_relvel);
    }

    return min_relvel;
}

bool solve_restitution_iteration(entt::registry &registry, scalar dt, unsigned individual_iterations) {
    auto body_view = registry.view<position, orientation, linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto restitution_view = registry.view<contact_manifold_with_restitution>();
    auto manifold_view = registry.view<contact_manifold>();

    // Solve manifolds in small groups, these groups being all manifolds connected
    // to one rigid body, usually a fast moving one. Ignore manifolds which are
    // separating, i.e. positive normal relative velocity. Intially, pick the
    // manifold with the highest penetration velocity (i.e. lowest normal relative
    // velocity) and select the fastest rigid body to start graph traversal. Traverse
    // the entity graph starting at that rigid body's node and visit the edges of
    // that node to collect the manifolds to be solved. Repeat to the other nodes
    // during traversal.

    // Find manifold with highest penetration velocity.
    auto min_relvel = EDYN_SCALAR_MAX;
    auto fastest_manifold_entity = entt::entity{entt::null};

    for (auto entity : restitution_view) {
        auto &manifold = manifold_view.get<contact_manifold>(entity);
        auto local_min_relvel = get_manifold_min_relvel(manifold, body_view, origin_view);

        if (local_min_relvel < min_relvel) {
            min_relvel = local_min_relvel;
            fastest_manifold_entity = entity;
        }
    }

    // This could be true if there are no contact points.
    if (fastest_manifold_entity == entt::null) {
        return true;
    }

    // In order to prevent bodies from bouncing forever, calculate a minimum
    // penetration velocity that must be met for the restitution impulse to
    // be applied.
    auto &fastest_manifold = manifold_view.get<contact_manifold>(fastest_manifold_entity);
    auto relvel_threshold = scalar(-0.005);

    if (min_relvel > relvel_threshold) {
        // All relative velocities are within threshold.
        return true;
    }

    // Reuse collections of rows to prevent a high number of allocations.
    auto normal_rows = std::vector<constraint_row>{};
    auto friction_row_pairs = std::vector<internal::contact_friction_row_pair>{};

    normal_rows.reserve(10);
    friction_row_pairs.reserve(10);

    auto solveManifolds = [&] (const std::vector<entt::entity> &manifold_entities) {
        normal_rows.clear();
        friction_row_pairs.clear();

        for (auto manifold_entity : manifold_entities) {
            auto &manifold = manifold_view.get<contact_manifold>(manifold_entity);

            auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(manifold.body[0]);
            auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(manifold.body[1]);

            auto originA = origin_view.contains(manifold.body[0]) ? origin_view.get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
            auto originB = origin_view.contains(manifold.body[1]) ? origin_view.get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

            // Create constraint rows for non-penetration constraints for each
            // contact point.
            for (size_t pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
                auto &cp = manifold.get_point(pt_idx);

                auto normal = cp.normal;
                auto pivotA = to_world_space(cp.pivotA, originA, ornA);
                auto pivotB = to_world_space(cp.pivotB, originB, ornB);
                auto rA = pivotA - posA;
                auto rB = pivotB - posB;

                auto &normal_row = normal_rows.emplace_back();
                normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
                normal_row.inv_mA = inv_mA; normal_row.inv_IA = inv_IA;
                normal_row.inv_mB = inv_mB; normal_row.inv_IB = inv_IB;
                normal_row.dvA = &dvA; normal_row.dwA = &dwA;
                normal_row.dvB = &dvB; normal_row.dwB = &dwB;
                normal_row.lower_limit = 0;
                normal_row.upper_limit = large_scalar;

                auto normal_options = constraint_row_options{};
                normal_options.restitution = cp.restitution;

                prepare_row(normal_row, normal_options, linvelA, angvelA, linvelB, angvelB);

                auto &friction_row_pair = friction_row_pairs.emplace_back();
                friction_row_pair.friction_coefficient = cp.friction;

                vector3 tangents[2];
                plane_space(normal, tangents[0], tangents[1]);

                for (auto i = 0; i < 2; ++i) {
                    auto &friction_row = friction_row_pair.row[i];
                    friction_row.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
                    friction_row.eff_mass = get_effective_mass(friction_row.J, inv_mA, inv_IA, inv_mB, inv_IB);
                    friction_row.rhs = -get_relative_speed(friction_row.J, linvelA, angvelA, linvelB, angvelB);
                }
            }
        }

        // Solve rows.
        for (unsigned iter = 0; iter < individual_iterations; ++iter) {
            for (size_t row_idx = 0; row_idx < normal_rows.size(); ++row_idx) {
                auto &normal_row = normal_rows[row_idx];
                auto delta_impulse = solve(normal_row);
                apply_impulse(delta_impulse, normal_row);

                auto &friction_row_pair = friction_row_pairs[row_idx];
                internal::solve_friction_row_pair(friction_row_pair, normal_row);
            }
        }

        // Persist applied impulses in a separate index because this cannot be
        // mixed with the regular constraint. It would apply these as the warm
        // starting impulse which will cause it to apply corrective impulses to
        // decelerate the rigid bodies which are separating.
        size_t row_idx = 0;

        for (auto manifold_entity : manifold_entities) {
            auto &manifold = manifold_view.get<contact_manifold>(manifold_entity);

            for (size_t pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
                auto &cp = manifold.get_point(pt_idx);
                auto &normal_row = normal_rows[row_idx];
                cp.normal_restitution_impulse = normal_row.impulse;

                auto &friction_row_pair = friction_row_pairs[row_idx];

                for (auto i = 0; i < 2; ++i) {
                    cp.friction_restitution_impulse[i] = friction_row_pair.row[i].impulse;
                }

                ++row_idx;
            }
        }

        // Apply delta velocities.
        for (auto manifold_entity : manifold_entities) {
            auto &manifold = manifold_view.get<contact_manifold>(manifold_entity);

            for (auto body_entity : manifold.body) {
                // There are duplicates among all manifold bodies but this
                // operation is idempotent since the delta velocity is set
                // to zero.
                auto [lv, av, dv, dw] = body_view.get<linvel, angvel, delta_linvel, delta_angvel>(body_entity);
                lv += dv;
                dv = vector3_zero;
                av += dw;
                dw = vector3_zero;
            }
        }
    };

    // Among the two rigid bodies in the manifold that is penetrating faster,
    // select the one that has the highest velocity.
    // Traversal is done over connecting nodes, thus ignore non-connecting nodes
    // (i.e. static and kinematic rigid bodies).
    auto &graph = registry.ctx<entity_graph>();
    entity_graph::index_type start_node_index;

    if (length_sqr(body_view.get<linvel>(fastest_manifold.body[0])) >
        length_sqr(body_view.get<linvel>(fastest_manifold.body[1]))) {
        auto &node0 = registry.get<graph_node>(fastest_manifold.body[0]);

        if (graph.is_connecting_node(node0.node_index)) {
            start_node_index = node0.node_index;
        } else {
            auto &node1 = registry.get<graph_node>(fastest_manifold.body[1]);
            EDYN_ASSERT(graph.is_connecting_node(node1.node_index));
            start_node_index = node1.node_index;
        }
    } else {
        auto &node1 = registry.get<graph_node>(fastest_manifold.body[1]);

        if (graph.is_connecting_node(node1.node_index)) {
            start_node_index = node1.node_index;
        } else {
            auto &node0 = registry.get<graph_node>(fastest_manifold.body[0]);
            EDYN_ASSERT(graph.is_connecting_node(node0.node_index));
            start_node_index = node0.node_index;
        }
    }

    std::vector<entt::entity> manifold_entities;

    graph.traverse_connecting_nodes(start_node_index, [&] (auto node_index) {
        graph.visit_edges(node_index, [&] (auto edge_index) {
            auto edge_entity = graph.edge_entity(edge_index);

            if (!manifold_view.contains(edge_entity)) return;

            auto &manifold = manifold_view.get<contact_manifold>(edge_entity);

            // Ignore manifolds which are not penetrating fast enough.
            auto local_min_relvel = get_manifold_min_relvel(manifold, body_view, origin_view);

            if (local_min_relvel < relvel_threshold) {
                manifold_entities.push_back(edge_entity);
            }
        });

        if (!manifold_entities.empty()) {
            solveManifolds(manifold_entities);
        }

        manifold_entities.clear();
    });

    return false;
}

void solve_restitution(entt::registry &registry, scalar dt) {
    auto &settings = registry.ctx<edyn::settings>();

    for (unsigned i = 0; i < settings.num_restitution_iterations; ++i) {
        if (solve_restitution_iteration(registry, dt, settings.num_individual_restitution_iterations)) {
            break;
        }
    }
}

}
