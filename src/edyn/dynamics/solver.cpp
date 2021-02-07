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
    , m_groups_changed(false)
    , m_constraint_rows_changed(false)
    , m_num_constraint_groups(0)
    , m_context(std::make_unique<solver_context>(registry.view<constraint_row_data>()))
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
    registry.on_construct<constraint_graph_node>().connect<&solver::on_construct_constraint_graph_node>(*this);
    registry.on_destroy<constraint_graph_node>().connect<&solver::on_destroy_constraint_graph_node>(*this);
    registry.on_construct<constraint_graph_edge>().connect<&solver::on_construct_constraint_graph_edge>(*this);
    registry.on_destroy<constraint_graph_edge>().connect<&solver::on_destroy_constraint_graph_edge>(*this);
    registry.on_construct<constraint_row>().connect<&solver::on_change_constraint_rows>(*this);
    registry.on_destroy<constraint_row>().connect<&solver::on_change_constraint_rows>(*this);
}

solver::~solver() = default;

void solver::on_construct_constraint_graph_node(entt::registry &, entt::entity entity) {
    m_new_nodes.insert(entity);
}

void solver::on_destroy_constraint_graph_node(entt::registry &registry, entt::entity entity) {

}

void solver::on_construct_constraint_graph_edge(entt::registry &, entt::entity entity) {
    m_new_edges.insert(entity);
}

void solver::on_destroy_constraint_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<constraint_graph_edge>(entity);

    if (edge.value == constraint_group::seam_group) return;

    EDYN_ASSERT(edge.value != constraint_group::null_group);
    auto size = m_group_sizes.at(edge.value);

    if (size > 1) {
        m_group_sizes.at(edge.value) = size - 1;
    } else {
        m_group_sizes.erase(edge.value);
    }

    m_groups_changed = true;
}

void solver::on_change_constraint_rows(entt::registry &registry, entt::entity entity) {
    m_constraint_rows_changed = true;
}

void solver::init_new_nodes() {
    if (m_new_nodes.empty()) return;

    if (m_group_sizes.empty()) {
        m_group_sizes.insert(std::make_pair(constraint_group::first_group, size_t{0}));
    }

    auto group_view = m_registry->view<constraint_group>();

    if (m_group_sizes.size() == 1) {
        auto group_value = m_group_sizes.begin()->first;

        for (auto entity : m_new_nodes) {
            group_view.get(entity).value = group_value;
        }

        m_new_nodes.clear();
        return;
    }

    // Make sure all new nodes have a null group.
    for (auto entity : m_new_nodes) {
        group_view.get(entity).value = constraint_group::null_group;
    }

    auto node_view = m_registry->view<island_node>();
    auto graph_edge_view = m_registry->view<constraint_graph_edge>();

    while (!m_new_nodes.empty()) {
        // Traverse graph trying to find another node which is already in a group
        // then assign that group to all nodes visited so far.
        std::vector<entt::entity> to_visit;
        to_visit.push_back(*m_new_nodes.begin());
        entity_set connected;

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();
            m_new_nodes.erase(entity);
            connected.insert(entity);

            auto &curr_node = node_view.get(entity);

            // Look through the connecting edge entities to find other nodes that
            // could be visited next.
            for (auto edge_entity : curr_node.entities) {
                if (!graph_edge_view.contains(edge_entity)) continue;

                auto &edge_node = node_view.get(edge_entity);
                EDYN_ASSERT(edge_node.entities.size() == 2);

                auto other_entity = entity;

                for (auto e : edge_node.entities) {
                    if (e == entity) continue;
                    other_entity = e;
                    break;
                }

                auto &group = group_view.get(other_entity);

                if (group.value != constraint_group::null_group) {
                    // A connecting node was found with a non-null group. Assign
                    // this group to all new entities visited so far.
                    for (auto e : connected) {
                        group_view.get(e).value = group.value;
                    }
                    connected.clear();
                } else if (m_new_nodes.count(other_entity) > 0) {
                    // Add entity to be visited if it has not been visited yet.
                    to_visit.push_back(other_entity);
                }
            }
        }
    }
}

void solver::init_new_edges() {
    if (m_new_edges.empty()) return;

    refresh_edges(m_new_edges.begin(), m_new_edges.end());

    m_new_edges.clear();

    m_groups_changed = true;
}

void solver::refresh_edge(entt::entity edge_entity, 
                          solver::node_view_t &node_view,
                          solver::group_view_t &group_view,
                          solver::constraint_view_t &constraint_view) {
    auto &edge_node = node_view.get(edge_entity);
    EDYN_ASSERT(edge_node.entities.size() == 2);

    auto it = edge_node.entities.begin();
    auto group0 = group_view.get(*it).value;
    std::advance(it, 1);
    auto group1 = group_view.get(*it).value;
    auto &edge_group = group_view.get(edge_entity);

    if (group0 == group1) {
        edge_group.value = group0;
        if (m_group_sizes.count(group0)) { 
            ++m_group_sizes.at(group0);
        } else {
            m_group_sizes.insert(std::make_pair(group0, size_t{0}));
        }
    } else {
        edge_group.value = constraint_group::seam_group;
    }

    m_registry->get<constraint_graph_edge>(edge_entity).value = edge_group.value;

    // Assign current group to all rows of this constraint graph edge.
    if (auto *manifold = m_registry->try_get<contact_manifold>(edge_entity); manifold) {
        for (size_t i = 0; i < manifold->num_points(); ++i) {
            auto contact_entity = manifold->point[i];
            group_view.get(contact_entity).value = edge_group.value;

            auto &con = constraint_view.get(contact_entity);
            for (size_t j = 0; j < con.num_rows(); ++j) {
                auto row_entity = con.row[j];
                group_view.get(row_entity).value = edge_group.value;
            }
        }
    } else if (constraint_view.contains(edge_entity)) {
        auto &con = constraint_view.get(edge_entity);
        for (size_t j = 0; j < con.num_rows(); ++j) {
            auto row_entity = con.row[j];
            group_view.get(row_entity).value = edge_group.value;
        }
    }
}

bool solver::is_constraint_graph_unbalanced() const {
    auto num_groups = m_group_sizes.size();

    // If there's only one group and the size of the group is not the
    // same as the number of edges, it means a group was destroyed and
    // there are still edges that are in the seam group.
    if (num_groups == 1 && m_registry->size<constraint_graph_edge>() > m_group_sizes.begin()->second) {
        return true;
    }

    if (num_groups < job_dispatcher::global().num_workers()) {
        for (auto &pair : m_group_sizes) {
            if (pair.second > m_max_group_size) {
                return true;
            }
        }
    }

    double average = 0;

    for (auto &pair : m_group_sizes) {
        average += (double)pair.second;
    }

    average /= (double)num_groups;
    double variance = 0;

    for (auto &pair : m_group_sizes) {
        auto d = (double)pair.second - average;
        variance += d * d;
    }

    variance /= (double)num_groups;

    return variance > m_max_group_size;
}

void solver::partite_constraint_graph() {
    auto node_view = m_registry->view<island_node>();
    auto graph_node_view = m_registry->view<constraint_graph_node>();
    auto graph_edge_view = m_registry->view<constraint_graph_edge>();
    auto node_entities = entity_set(graph_node_view.begin(), graph_node_view.end());

    auto group_view = m_registry->view<constraint_group>();
    group_view.each([] (constraint_group &group) {
        group.value = constraint_group::null_group;
    });

    const size_t nodes_per_worker = std::ceil(graph_node_view.size() / job_dispatcher::global().num_workers());
    const size_t desired_group_size = std::max(m_max_group_size / 2, nodes_per_worker);
    auto current_group = constraint_group::first_group;
    size_t group_size = 0;
    std::vector<entt::entity> to_visit;

    // Starting at an arbitrary graph node, traverse the graph collecting nodes 
    // into the current group until it reaches the desired group size. Then move
    // over to the next group and continue traversal again collecting nodes into
    // the current group, and so on...
    while (!node_entities.empty()) {
        auto node_entity = *node_entities.begin();
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();
            node_entities.erase(entity);

            auto &group = group_view.get(entity);

            if (group.value != constraint_group::null_group) continue;

            group.value = current_group;

            auto &curr_node = node_view.get(entity);

            for (auto edge_entity : curr_node.entities) {
                if (!graph_edge_view.contains(edge_entity)) continue;

                auto &edge_node = node_view.get(edge_entity);
                EDYN_ASSERT(edge_node.entities.size() == 2);

                // Increment group size for each edge.
                ++group_size;

                // If the group is bigger than the desired size, start a new group.
                if (group_size >= desired_group_size) {
                    ++current_group;
                    group_size = 0;
                }

                // Initialize other entity with current entity to handle cycles.
                auto other_entity = entity;

                for (auto e : edge_node.entities) {
                    if (e == entity) continue;
                    other_entity = e;
                    break;
                }

                auto visited = node_entities.count(other_entity) == 0;
                if (!visited) {
                    to_visit.push_back(other_entity);
                }
            }
        }
    }

    m_num_constraint_groups = current_group;

    m_group_sizes.clear();

    auto edge_view = m_registry->view<constraint_graph_edge>();
    refresh_edges(edge_view.begin(), edge_view.end());

    m_constraint_rows_changed = true;
}

void solver::sort_constraint_rows() {
    auto group_view = m_registry->view<constraint_group>();
    // Sort constraint rows by group.
    m_registry->sort<constraint_row>([&group_view] (entt::entity lhs, entt::entity rhs) {
        return group_view.get(lhs).value < group_view.get(rhs).value;
    });
    m_registry->sort<constraint_row_data, constraint_row>();
}

void solver::prepare_constraint_graph() {
    if (m_groups_changed) {
        if (is_constraint_graph_unbalanced()) {
            partite_constraint_graph();
        }
        m_groups_changed = false;
    }

    // Constraint rows do not need to be sorted and the context does not need
    // to be updated if the solver is not parallelizable since these changes
    // would not affect the serial solver.
    if (m_constraint_rows_changed && parallelizable()) {
        sort_constraint_rows();
        m_constraint_rows_changed = false;

        // Calculate the range of each constraint group in the sorted
        // `constraint_row` view.
        auto group_view = m_registry->view<constraint_group>();
        auto row_view = m_registry->view<constraint_row>();
        EDYN_ASSERT(group_view.size() > 1);

        m_context->ranges.clear();
        m_context->ranges.push_back(0);

        auto group_value = constraint_group::first_group;
        m_num_constraint_groups = 0;
        size_t index = 0;

        // Loop through the sorted constraint row data view until the seam group
        // is reached (it is always the last).
        while (true) {
            auto curr_value = group_view.get(row_view[index]).value;
            EDYN_ASSERT(curr_value != constraint_group::null_group);

            if (curr_value == group_value) {
                ++index;
                continue;
            }

            m_context->ranges.push_back(index);
            ++m_num_constraint_groups;

            if (curr_value != constraint_group::seam_group) {
                group_value = curr_value;
            } else {
                break;
            }
        }
    }
}

bool solver::parallelizable() const {
    return m_num_constraint_groups > 1;
}

void solver::update(scalar dt) {
    init_new_nodes();
    init_new_edges();
    prepare_constraint_graph();

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
    init_new_nodes();
    init_new_edges();
    prepare_constraint_graph();

    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

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

    m_context->completion = completion;

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

    // Solve the seam constraint group.
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