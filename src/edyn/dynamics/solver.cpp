#include "edyn/dynamics/solver.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entt.hpp>
#include <type_traits>

namespace edyn {

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

void prepare_row(const constraint_row &row, constraint_row_data &data,
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

void update_inertia(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>();
    view.each([] (orientation& orn, inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = basis * inv_I * transpose(basis);
    });
}

template<typename C>
void update_impulse(entt::registry &registry, row_cache &cache, size_t &con_idx, size_t &row_idx) {
    auto con_view = registry.view<C>();
    con_view.each([&] (entt::entity entity, C &con) {
        auto num_rows = cache.con_num_rows[con_idx];
        for (size_t i = 0; i < num_rows; ++i) {
            con.impulse[i] = cache.con_datas[row_idx + i].impulse;
        }

        row_idx += num_rows;
        ++con_idx;
    });
}

template<typename... Cs>
void update_impulse_tuple(std::tuple<Cs...>, entt::registry &registry, row_cache &cache, size_t &con_idx, size_t &row_idx) {
    (update_impulse<Cs>(registry, cache, con_idx, row_idx), ...);
}

void update_impulses(entt::registry &registry, row_cache &cache) {
    size_t con_idx = 0;
    size_t row_idx = 0;
    update_impulse_tuple(constraints_tuple_t{}, registry, cache, con_idx, row_idx);
}

solver::solver(entt::registry &registry) 
    : m_registry(&registry)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
}

solver::~solver() = default;

void solver::update(scalar dt) {
    auto &registry = *m_registry;

    m_row_cache.clear();

    // Apply forces and acceleration.
    integrate_linacc(registry, dt);
    apply_gravity(registry, dt);

    // Setup constraints.
    prepare_constraints(registry, m_row_cache, dt);

    EDYN_ASSERT(m_row_cache.con_rows.size() == m_row_cache.con_datas.size());

    // Solve constraints.
    for (uint32_t i = 0; i < iterations; ++i) {
        // Prepare constraints for iteration.
        iterate_constraints(registry, m_row_cache, dt);

        // Solve rows.
        for (auto &data : m_row_cache.con_datas) {
            auto delta_impulse = solve(data);
            apply_impulse(delta_impulse, data);
        }
    }

    // Apply constraint velocity correction.
    auto vel_view = registry.view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>();
    vel_view.each([] (linvel &v, angvel &w, delta_linvel &dv, delta_angvel &dw) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    });

    // Assign applied impulses.
    update_impulses(registry, m_row_cache);

    // Integrate velocities to obtain new transforms.
    integrate_linvel(registry, dt);
    integrate_angvel(registry, dt);

    // Update AABBs after transforms change.
    update_aabbs(registry);
    
    // Update rotated vertices of convex meshes after rotations change.
    update_rotated_meshes(registry);

    // Update world-space moment of inertia.
    update_inertia(registry);
}

}
