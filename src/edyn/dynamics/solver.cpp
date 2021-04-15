#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include <entt/entt.hpp>

namespace edyn {

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

static
void prepare(const constraint_row &row, constraint_row_data &data,
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

void update_inertia(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>();
    view.each([] (orientation& orn, inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = basis * inv_I * transpose(basis);
    });
}

solver::solver(entt::registry &registry) 
    : m_registry(&registry)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
}

solver::~solver() = default;

template<typename T>
class has_iteration {
    using yes = char[1];
    using no = char[2];
    template<typename U> static yes & test(decltype(&U::iteration));
    template<typename U> static no & test(...);
public:
    enum { value = sizeof(test<T>(0)) == sizeof(yes) };
};

void solver::update(scalar dt) {
    auto &registry = *m_registry;

    m_row_cache.clear();

    // Apply forces and acceleration.
    integrate_linacc(registry, dt);
    apply_gravity(registry, dt);

    // Setup constraints.
    auto body_view = registry.view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = registry.view<constraint>();

    size_t row_idx = 0;
    con_view.each([&] (entt::entity entity, constraint &con) {
        auto prev_size = m_row_cache.con_rows.size();
        std::visit([&] (auto &&c) {
            c.prepare(entity, con, registry, m_row_cache, dt);
        }, con.var);
        auto curr_size = m_row_cache.con_rows.size();
        auto num_rows = curr_size - prev_size;
        m_row_cache.con_num_rows.push_back(num_rows);

        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(con.body[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(con.body[1]);

        for (size_t i = 0; i < num_rows; ++i) {
            auto j = i + row_idx;
            const auto &row = m_row_cache.con_rows[j];
            auto &data = m_row_cache.con_datas[j];

            data.inv_mA = inv_mA;
            data.inv_mB = inv_mB;
            data.inv_IA = inv_IA;
            data.inv_IB = inv_IB;

            data.dvA = &dvA;
            data.dvB = &dvB;
            data.dwA = &dwA;
            data.dwB = &dwB;

            data.impulse = con.impulse[i];

            prepare(row, data, linvelA, linvelB, angvelA, angvelB);
            warm_start(data);
        }

        row_idx += num_rows;
    });

    EDYN_ASSERT(m_row_cache.con_rows.size() == m_row_cache.con_datas.size());

    // Solve constraints.
    for (uint32_t i = 0; i < iterations; ++i) {
        // Prepare constraints for iteration.
        size_t row_idx = 0;
        size_t con_idx = 0;

        con_view.each([&] (entt::entity entity, constraint &con) {
            std::visit([&] (auto &&c) {
                using ConstraintType = std::decay_t<decltype(c)>;
                if constexpr(has_iteration<ConstraintType>::value) {
                    c.iteration(entity, con, registry, m_row_cache, row_idx, dt);
                }
            }, con.var);
            row_idx += m_row_cache.con_num_rows[con_idx++];
        });

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
    size_t data_idx = 0;
    size_t con_idx = 0;
    con_view.each([&] (entt::entity entity, constraint &con) {
        auto num_rows = m_row_cache.con_num_rows[con_idx];
        for (size_t i = 0; i < num_rows; ++i) {
            con.impulse[i] = m_row_cache.con_datas[data_idx + i].impulse;
        }

        data_idx += num_rows;
        ++con_idx;
    });

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
