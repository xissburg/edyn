#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/orientation.hpp"
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
#include "edyn/util/rigidbody.hpp"
#include "edyn/comp/constraint_group.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include <entt/entt.hpp>
#include <atomic>

namespace edyn {

// Solver context for the parallel solver iterations.
struct solver_context {
    using row_data_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, constraint_row_data>;
    row_data_view_t data_view;
    std::vector<size_t> ranges;
    std::atomic<uint32_t> group_index;
    std::atomic<unsigned> counter;
    job completion;

    solver_context(const row_data_view_t &view)
        : data_view(view)
    {}
};

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

static
void prepare(constraint_row &row, constraint_row_data &data,
             const vector3 &linvelA, const vector3 &linvelB,
             const vector3 &angvelA, const vector3 &angvelB) {
    auto J_invM_JT = dot(data.J[0], data.J[0]) * data.inv_mA +
                     dot(data.inv_IA * data.J[1], data.J[1]) +
                     dot(data.J[2], data.J[2]) * data.inv_mB +
                     dot(data.inv_IB * data.J[3], data.J[3]);
    data.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(data.J[0], linvelA) + 
                  dot(data.J[1], angvelA) +
                  dot(data.J[2], linvelB) +
                  dot(data.J[3], angvelB);
    
    auto restitution = restitution_curve(row.restitution, relvel);
    data.rhs = -(row.error * row.erp + relvel * (1 + restitution));
}

static
void apply_impulse(scalar impulse, constraint_row_data &data) {
    // Apply linear impulse.
    *data.dvA += data.inv_mA * data.J[0] * impulse;
    *data.dvB += data.inv_mB * data.J[2] * impulse;

    // Apply angular impulse.
    *data.dwA += data.inv_IA * data.J[1] * impulse;
    *data.dwB += data.inv_IB * data.J[3] * impulse;
}

static
void warm_start(constraint_row_data &data) {
    apply_impulse(data.impulse, data);
}

static
scalar solve(constraint_row_data &data) {
    auto delta_relvel = dot(data.J[0], *data.dvA) + 
                        dot(data.J[1], *data.dwA) +
                        dot(data.J[2], *data.dvB) +
                        dot(data.J[3], *data.dwB);
    auto delta_impulse = (data.rhs - delta_relvel) * data.eff_mass;
    auto impulse = data.impulse + delta_impulse;

    if (impulse < data.lower_limit) {
        delta_impulse = data.lower_limit - data.impulse;
        data.impulse = data.lower_limit;
    } else if (impulse > data.upper_limit) {
        delta_impulse = data.upper_limit - data.impulse;
        data.impulse = data.upper_limit;
    } else {
        data.impulse = impulse;
    }

    return delta_impulse;
}

static
void solver_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<solver_context *>(ctx_ptr);

    while (true) {
        auto group = ctx->group_index.fetch_add(1, std::memory_order_relaxed);

        if (group + 1 >= ctx->ranges.size()) {
            break;
        }

        auto first = ctx->ranges[group];
        auto last = ctx->ranges[group + 1];

        for (size_t i = first; i < last; ++i) {
            auto &data = ctx->data_view.get(ctx->data_view[i]);
            auto delta_impulse = solve(data);
            apply_impulse(delta_impulse, data);
        }

        auto remaining = ctx->counter.fetch_sub(1, std::memory_order_relaxed) - 1;

        if (remaining == 0) {
            job_dispatcher::global().async(ctx->completion);
            break;
        }
    }
}

void update_inertia(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>(entt::exclude<disabled_tag>);
    view.each([] (auto, orientation& orn, inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = scale(basis, inv_I) * transpose(basis);
    });
}

solver::solver(entt::registry &registry) 
    : m_registry(&registry)
    , m_constraints_changed(false)
    , m_num_constraint_groups(0)
    , m_context(std::make_unique<solver_context>(registry.view<constraint_row_data>()))
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
    registry.on_construct<constraint_graph_node>().connect<&solver::on_change_constraint_graph>(*this);
    registry.on_destroy<constraint_graph_node>().connect<&solver::on_change_constraint_graph>(*this);
    registry.on_construct<constraint_graph_edge>().connect<&solver::on_change_constraint_graph>(*this);
    registry.on_destroy<constraint_graph_edge>().connect<&solver::on_change_constraint_graph>(*this);
}

solver::~solver() = default;

void solver::on_change_constraint_graph(entt::registry &registry, entt::entity entity) {
    m_constraints_changed = true;
}

void solver::init_new_rows() {
    /*if (m_new_rows.empty()) return;

    for (auto entity : m_new_rows) {
        
    }

    m_new_rows.clear();*/
}

void solver::partite_constraint_graph() {
    auto node_view = m_registry->view<island_node>();
    auto graph_node_view = m_registry->view<constraint_graph_node>();
    auto graph_edge_view = m_registry->view<constraint_graph_edge>();
    auto constraint_view = m_registry->view<constraint>();
    auto node_entities = entity_set(graph_node_view.begin(), graph_node_view.end());

    auto group_view = m_registry->view<constraint_group>();
    group_view.each([] (constraint_group &group) {
        group.value = 0;
    });

    constraint_group::value_t current_group = 1;
    const auto desired_group_size = graph_edge_view.size() / job_dispatcher::global().num_workers();

    while (!node_entities.empty()) {
        size_t group_size = 0;
        entity_set connected;
        std::vector<entt::entity> to_visit;

        auto node_entity = *node_entities.begin();
        to_visit.push_back(node_entity);

        group_view.get(node_entity).value = current_group;

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();

            node_entities.erase(entity);

            auto &curr_node = node_view.get(entity);

            for (auto edge_entity : curr_node.entities) {
                if (!graph_edge_view.contains(edge_entity)) continue;

                auto already_visited = connected.count(edge_entity);
                if (already_visited) continue;

                connected.insert(edge_entity);

                auto &edge_node = node_view.get(edge_entity);
                EDYN_ASSERT(edge_node.entities.size() == 2);

                entt::entity other_entity;

                for (auto e : edge_node.entities) {
                    if (e == entity) continue;
                    other_entity = e;
                    break;
                }

                auto edge_group_value = current_group;

                auto &other_group = group_view.get(other_entity);

                if (other_group.value == 0) {
                    to_visit.push_back(other_entity);
                    group_view.get(other_entity).value = current_group;
                } else if (other_group.value != current_group) {
                    edge_group_value = constraint_group::stitch_group;
                }

                group_view.get(edge_entity).value = edge_group_value;

                // Assign current group to all rows of this constraint graph edge.
                if (auto *manifold = m_registry->try_get<contact_manifold>(edge_entity); manifold) {
                    for (size_t i = 0; i < manifold->num_points(); ++i) {
                        auto contact_entity = manifold->point[i];
                        auto &con = constraint_view.get(contact_entity);
                        group_view.get(contact_entity).value = edge_group_value;

                        for (size_t j = 0; j < con.num_rows(); ++j) {
                            auto row_entity = con.row[j];
                            group_view.get(row_entity).value = edge_group_value;
                        }
                    }
                }

                // If the edge is in the current group, increment the size.
                if (edge_group_value == current_group) {
                    ++group_size;

                    // If the group is bigger than the desired size, start a new group.
                    if (group_size > desired_group_size) {
                        ++current_group;
                        group_size = 0;
                    }
                }
            }
        }

        ++current_group;
    }

    m_num_constraint_groups = current_group - 1;

    // Sort constraint rows by group.
    m_registry->sort<constraint_group>([] (const auto &lhs, const auto &rhs) {
        EDYN_ASSERT(lhs.value != 0 && rhs.value != 0);
        return lhs.value < rhs.value;
    });
    m_registry->sort<constraint_row_data, constraint_group>();
}

bool solver::parallelizable() const {
    return m_num_constraint_groups > 1;
}

void solver::update(scalar dt) {
    init_new_rows();

    if (m_constraints_changed) {
        partite_constraint_graph();
        m_constraints_changed = false;
    }

    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, constraint_row_data>(entt::exclude<disabled_tag>);
    auto data_view = m_registry->view<constraint_row_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    row_view.each([&] (constraint_row &row, constraint_row_data &data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        data.inv_mA = inv_mA;
        data.inv_mB = inv_mB;
        data.inv_IA = inv_IA;
        data.inv_IB = inv_IB;

        data.dvA = &dvA;
        data.dvB = &dvB;
        data.dwA = &dwA;
        data.dwB = &dwB;

        prepare(row, data, linvelA, linvelB, angvelA, angvelB);
        warm_start(data);
    });

    // Solve constraints.
    for (uint32_t i = 0; i < iterations; ++i) {
        // Prepare constraints for iteration.
        con_view.each([&] (entt::entity entity, constraint &con) {
            std::visit([&] (auto &&c) {
                c.update(solver_stage_value_t<solver_stage::iteration>{}, entity, con, *m_registry, dt);
            }, con.var);
        });

        // Solve rows.
        data_view.each([&] (constraint_row_data &data) {
            auto delta_impulse = solve(data);
            apply_impulse(delta_impulse, data);
        });
    }

    // Apply constraint velocity correction.
    auto vel_view = m_registry->view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude<disabled_tag>);
    vel_view.each([] (linvel &v, angvel &w, delta_linvel &dv, delta_angvel &dw) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*m_registry, dt);
    integrate_angvel(*m_registry, dt);
    update_aabbs(*m_registry);
    
    // Update world-space moment of inertia.
    update_inertia(*m_registry);
}

void solver::start_async_update(scalar dt, const job &completion) {
    init_new_rows();

    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    if (m_constraints_changed) {
        partite_constraint_graph();
        m_constraints_changed = false;
    }

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, constraint_row_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    row_view.each([&] (constraint_row &row, constraint_row_data &data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        data.inv_mA = inv_mA;
        data.inv_mB = inv_mB;
        data.inv_IA = inv_IA;
        data.inv_IB = inv_IB;

        data.dvA = &dvA;
        data.dvB = &dvB;
        data.dwA = &dwA;
        data.dwB = &dwB;

        prepare(row, data, linvelA, linvelB, angvelA, angvelB);
        warm_start(data);
    });

    m_state.dt = dt;
    m_state.iteration = 0;

    // Prepare solver context to start async iterations. Calculate the range of 
    // each constraint group in the sorted `constraint_group` view.
    auto group_view = m_registry->view<constraint_group>();
    auto data_view = m_registry->view<constraint_row_data>();
    EDYN_ASSERT(group_view.size() > 1);

    m_context->completion = completion;
    m_context->ranges.clear();
    m_context->ranges.push_back(0);

    auto group_value = group_view.get(group_view[0]).value;
    m_num_constraint_groups = 0;
    size_t index = 1;

    // Loop through the sorted constraint row data view until the stitch group
    // is reached (it is always the last).
    while (true) {
        auto curr_value = group_view.get(data_view[index]).value;

        if (curr_value == group_value) {
            ++index;
            continue;
        }

        m_context->ranges.push_back(index);
        ++m_num_constraint_groups;

        if (curr_value != constraint_group::stitch_group) {
            group_value = curr_value;
        } else {
            break;
        }
    }

    // Dispatch first iteration.
    run_async_iteration();
}

void solver::run_async_iteration() {
    // Prepare constraints for iteration.
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::iteration>{}, entity, con, *m_registry, m_state.dt);
        }, con.var);
    });

    // Solve the stitch constraint group.
    auto data_view = m_registry->view<constraint_row_data>();
    auto first = m_context->ranges.back();
    auto last = data_view.size();

    for (size_t i = first; i < last; ++i) {
        auto &data = data_view.get(data_view[i]);
        auto delta_impulse = solve(data);
        apply_impulse(delta_impulse, data);
    }

    // Dispatch disjoint groups to be solved in parallel.
    dispatch_solver_job();
}

void solver::dispatch_solver_job() {
    m_context->counter.store(m_num_constraint_groups, std::memory_order_relaxed);
    m_context->group_index.store(0, std::memory_order_relaxed);

    auto solver_job = job();
    solver_job.func = &solver_job_func;
    auto archive = fixed_memory_output_archive(solver_job.data.data(), solver_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(m_context.get());
    archive(ctx_ptr);

    for (size_t i = 0; i < m_num_constraint_groups; ++i) {
        job_dispatcher::global().async(solver_job);
    }
}

bool solver::continue_async_update() {
    ++m_state.iteration;

    if (m_state.iteration < iterations) {
        run_async_iteration();
        return false;
    } else {
        finish_async_update();
        return true;
    }
}

void solver::finish_async_update() {
    // Apply constraint velocity correction.
    auto vel_view = m_registry->view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude<disabled_tag>);
    vel_view.each([] (linvel &v, angvel &w, delta_linvel &dv, delta_angvel &dw) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*m_registry, m_state.dt);
    integrate_angvel(*m_registry, m_state.dt);
    update_aabbs(*m_registry);
    
    // Update world-space moment of inertia.
    update_inertia(*m_registry);
}

}