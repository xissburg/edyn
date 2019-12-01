#include <entt/entity/view.hpp>
#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/dynamics/solver_stage.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

void on_construct_constraint(entt::entity entity, entt::registry &registry, constraint &con) {
    auto &rel = registry.get<relation>(entity);

    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, con, rel, registry, 0);
    }, con.var);
}

void on_destroy_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    for (auto e : con.row) {
        if (e != entt::null && registry.valid(e)) {
            registry.destroy(e);
        }
    }
}

void on_construct_relation(entt::entity entity, entt::registry &registry, relation &rel) {
    // Allow the related entities to refer to their relations.
    registry.get_or_assign<relation_container>(rel.entity[0]).entities.push_back(entity);
    registry.get_or_assign<relation_container>(rel.entity[1]).entities.push_back(entity);

    // Find all islands involved in this new relation.
    std::vector<entt::entity> island_ents;

    for (auto ent : rel.entity) {
        auto node = registry.try_get<island_node>(ent);
        if (node) {
            island_ents.push_back(node->island_entity);
        }
    }

    // All entities are in the same island nothing needs to be done.
    if (island_ents.size() < 2) {
        return;
    }

    // Merge all into one island.
    // TODO: choose island with most nodes.
    auto island_ent = island_ents[0];

    for (size_t i = 1; i < island_ents.size(); ++i) {
        auto other_ent = island_ents[i];

        if (other_ent != island_ent) {
            auto &isle = registry.get<island>(island_ent);
            auto &other_isle = registry.get<island>(other_ent);

            for (auto ent : other_isle.entities) {
                isle.entities.push_back(ent);

                auto &node = registry.get<island_node>(ent);
                node.island_entity = island_ent;

                // Wake up if sleeping.
                if (registry.has<sleeping_tag>(ent)) {
                    registry.remove<sleeping_tag>(ent);
                }
            }

            // Destroy island entity.
            registry.destroy(other_ent);
        }
    }
}

void on_destroy_relation(entt::entity entity, entt::registry &registry) {
    // Perform graph-walks using the entities in the destroyed relation as the
    // starting point (ignoring this destroyed relation, of course). Store the 
    // entities visited in each case in a set. If all sets are equal, it means
    // the island has not been broken and nothing needs to be done. If the sets
    // are different, destroy this island and create news islands for each set.
    // Update `island_node`s to point the new islands.

    auto &rel = registry.get<relation>(entity);

    // Remove the destroyed relation from the `relation_container` of the
    // related entities. Empty containers are removed at the end.
    for (size_t i = 0; i < max_relations; ++i) {
        auto &container = registry.get<relation_container>(rel.entity[i]);
        auto it = std::find(container.entities.begin(), container.entities.end(), entity);
        std::swap(*it, *(container.entities.end() - 1));
        container.entities.pop_back();
    }

    // Store all entities found while walking the graph from each starting
    // point.
    std::array<std::vector<entt::entity>, max_relations> connected_entities;

    // Walk the graph starting from each entity in the destroyed relation.
    for (size_t i = 0; i < max_relations; ++i) {
        if (!registry.has<dynamic_tag>(rel.entity[i])) {
            continue;
        }

        std::vector<entt::entity> visit_me;
        visit_me.push_back(rel.entity[i]);

        while (!visit_me.empty()) {
            auto ent = visit_me.back();
            visit_me.pop_back();

            if (std::find(connected_entities[i].begin(), connected_entities[i].end(), ent) == connected_entities[i].end()) {
                connected_entities[i].push_back(ent);
            } else {
                continue; // Already visited.
            }

            // Grab neighboring entities to visit.
            auto &container = registry.get<relation_container>(ent);

            for (auto rel_ent : container.entities) {
                // Skip the destroyed relation.
                if (rel_ent == entity) {
                    continue;
                }

                auto &rel = registry.get<relation>(rel_ent);
                
                if (rel.entity[0] != ent && registry.has<dynamic_tag>(rel.entity[0])) {
                    visit_me.push_back(rel.entity[0]);
                }

                if (rel.entity[1] != ent && registry.has<dynamic_tag>(rel.entity[1])) {
                    visit_me.push_back(rel.entity[1]);
                }
            }
        }
    }

    // If the sets are different, split island.
    for (size_t i = 0; i < max_relations; ++i) {
        std::sort(connected_entities[i].begin(), connected_entities[i].end());
    }

    if (connected_entities[0] != connected_entities[1]) {
        // Destroy original island containing both entities in this relation.
        auto &node = registry.get<island_node>(rel.entity[0]);
        registry.destroy(node.island_entity);

        // Create one new island for the first set.
        auto [island_ent, isle] = registry.create<island>();
        isle.entities = std::move(connected_entities[0]);

        // Update all nodes in the first set to point to the new island.
        for (auto ent : isle.entities) {
            auto &node = registry.get<island_node>(ent);
            node.island_entity = island_ent;
            
            if (registry.has<sleeping_tag>(ent)) {
                registry.remove<sleeping_tag>(ent);
            }
        }

        // Create another island for the second set.
        auto [other_island_ent, other_isle] = registry.create<island>();    
        other_isle.entities = std::move(connected_entities[1]);

        // Update all nodes in the second set.
        for (auto ent : other_isle.entities) {
            auto &node = registry.get<island_node>(ent);
            node.island_entity = other_island_ent;
            
            if (registry.has<sleeping_tag>(ent)) {
                registry.remove<sleeping_tag>(ent);
            }
        }
    } else {
        // Island survives. Wake everyone up though. They might have work to do
        // now.
        for (auto ent : connected_entities[0]) {            
            if (registry.has<sleeping_tag>(ent)) {
                registry.remove<sleeping_tag>(ent);
            }
        }
    }

    // Remove empty `relation_container`s.
    for (size_t i = 0; i < max_relations; ++i) {
        auto &container = registry.get<relation_container>(rel.entity[i]);
        if (container.entities.empty()) {
            registry.remove<relation_container>(rel.entity[i]);
        }
    }
}

void on_destroy_linvel(entt::entity entity, entt::registry &registry) {
    if (registry.has<delta_linvel>(entity)) {
        registry.remove<delta_linvel>(entity);
    }
}

void on_destroy_angvel(entt::entity entity, entt::registry &registry) {
    if (registry.has<delta_angvel>(entity)) {
        registry.remove<delta_angvel>(entity);
    }
}

void prepare(constraint_row &row, 
             scalar inv_mA, scalar inv_mB, 
             const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
             const vector3 &linvelA, const vector3 &linvelB,
             const vector3 &angvelA, const vector3 &angvelB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * inv_mA +
                     dot(inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * inv_mB +
                     dot(inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) + 
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB);
    row.rhs = -(row.error + relvel * (1 + row.restitution));
}

void warm_start(constraint_row &row, 
                scalar inv_mA, scalar inv_mB, 
                const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
                delta_linvel &dvA, delta_linvel &dvB,
                delta_angvel &dwA, delta_angvel &dwB) {
    dvA += inv_mA * row.J[0] * row.impulse;
    dwA += inv_IA * row.J[1] * row.impulse;
    dvB += inv_mB * row.J[2] * row.impulse;
    dwB += inv_IB * row.J[3] * row.impulse;
}

template<typename MassInvView, typename DeltaView>
void solve(constraint_row &row,
           MassInvView &mass_inv_view,
           DeltaView &delta_view) {
    auto [dvA, dwA] = delta_view.template get<delta_linvel, delta_angvel>(row.entity[0]);
    auto [dvB, dwB] = delta_view.template get<delta_linvel, delta_angvel>(row.entity[1]);

    auto delta_relvel = dot(row.J[0], dvA) + 
                        dot(row.J[1], dwA) +
                        dot(row.J[2], dvB) +
                        dot(row.J[3], dwB);
    auto delta_impulse = (row.rhs - delta_relvel) * row.eff_mass;
    auto impulse = row.impulse + delta_impulse;

    if (impulse < row.lower_limit) {
        delta_impulse = row.lower_limit - row.impulse;
        row.impulse = row.lower_limit;
    } else if (impulse > row.upper_limit) {
        delta_impulse = row.upper_limit - row.impulse;
        row.impulse = row.upper_limit;
    } else {
        row.impulse = impulse;
    }

    auto [inv_mA, inv_IA] = mass_inv_view.template get<mass_inv, inertia_world_inv>(row.entity[0]);
    auto [inv_mB, inv_IB] = mass_inv_view.template get<mass_inv, inertia_world_inv>(row.entity[1]);

    // Apply impulse.
    dvA += inv_mA * row.J[0] * delta_impulse;
    dwA += inv_IA * row.J[1] * delta_impulse;
    dvB += inv_mB * row.J[2] * delta_impulse;
    dwB += inv_IB * row.J[3] * delta_impulse;
}

void update_inertia(entt::registry &registry) {
    auto view = registry.view<const orientation, const inertia_inv, inertia_world_inv>();
    view.each([] (auto, const orientation& orn, const inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = scale(basis, inv_I) * transpose(basis);
    });
}

solver::solver(entt::registry &reg) 
    : registry(&reg)
{
    connections.push_back(reg.on_construct<constraint>().connect<&on_construct_constraint>());
    connections.push_back(reg.on_destroy<constraint>().connect<&on_destroy_constraint>());

    connections.push_back(reg.on_construct<relation>().connect<&on_construct_relation>());
    connections.push_back(reg.on_destroy<relation>().connect<&on_destroy_relation>());

    connections.push_back(reg.on_construct<linvel>().connect<&entt::registry::assign<delta_linvel>>(reg));
    connections.push_back(reg.on_destroy<linvel>().connect<&on_destroy_linvel>());

    connections.push_back(reg.on_construct<angvel>().connect<&entt::registry::assign<delta_angvel>>(reg));
    connections.push_back(reg.on_destroy<angvel>().connect<&on_destroy_angvel>());
}

void solver::update(scalar dt) {
    // Apply forces and acceleration.
    integrate_linacc(*registry, dt);
    apply_gravity(*registry, dt);

    // Setup constraints.
    auto mass_inv_view = registry->view<mass_inv, inertia_world_inv>();
    auto vel_view = registry->view<linvel, angvel>();
    auto delta_view = registry->view<delta_linvel, delta_angvel>();

    auto con_view = registry->view<const relation, constraint>();
    con_view.each([&] (auto, const relation &rel, constraint &con) {
        auto [inv_mA, inv_IA] = mass_inv_view.get<mass_inv, inertia_world_inv>(rel.entity[0]);
        auto [inv_mB, inv_IB] = mass_inv_view.get<mass_inv, inertia_world_inv>(rel.entity[1]);
        auto [linvelA, angvelA] = vel_view.get<linvel, angvel>(rel.entity[0]);
        auto [linvelB, angvelB] = vel_view.get<linvel, angvel>(rel.entity[1]);
        auto [dvA, dwA] = delta_view.template get<delta_linvel, delta_angvel>(rel.entity[0]);
        auto [dvB, dwB] = delta_view.template get<delta_linvel, delta_angvel>(rel.entity[1]);

        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, con, rel, *registry, dt);

            for (size_t i = 0; i < con.num_rows; ++i) {
                auto &row = registry->get<constraint_row>(con.row[i]);
                prepare(row, inv_mA, inv_mB, inv_IA, inv_IB, linvelA, linvelB, angvelA, angvelB);
                warm_start(row, inv_mA, inv_mB, inv_IA, inv_IB, dvA, dvB, dwA, dwB);
            }
        }, con.var);
    });

    // Solve constraints.
    auto row_view = registry->view<constraint_row>();

    for (uint32_t i = 0; i < iterations; ++i) {
        con_view.each([&] (auto, const relation &rel, constraint &con) {
            std::visit([&] (auto &&c) {
                c.update(solver_stage_value_t<solver_stage::iteration>{}, con, rel, *registry, dt);
            }, con.var);
        });

        row_view.each([&] (auto, auto &row) {
            solve(row, mass_inv_view, delta_view);
        });
    }

    // Apply constraint velocity correction.
    registry->view<linvel, delta_linvel>().each([] (auto, auto &vel, auto &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    registry->view<angvel, delta_angvel>().each([] (auto, auto &vel, auto &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*registry, dt);
    integrate_angvel(*registry, dt);

    // Update world-space moment of inertia.
    update_inertia(*registry);
}

}