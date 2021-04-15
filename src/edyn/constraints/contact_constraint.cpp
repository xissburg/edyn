#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/array.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entt.hpp>

namespace edyn {

void contact_constraint::prepare(entt::entity entity, const constraint &con, 
                                 entt::registry &registry, row_cache &cache, 
                                 scalar dt) {
    auto body_view = registry.view<position, orientation, linvel, angvel>();

    auto [posA, ornA, linvelA, angvelA] = body_view.get<position, orientation, linvel, angvel>(con.body[0]);
    auto [posB, ornB, linvelB, angvelB] = body_view.get<position, orientation, linvel, angvel>(con.body[1]);

    auto &cp = registry.get<contact_point>(entity);

    auto rA = rotate(ornA, cp.pivotA);
    auto rB = rotate(ornB, cp.pivotB);
    auto normal = rotate(ornB, cp.normalB);

    auto vA = linvelA + cross(angvelA, rA);
    auto vB = linvelB + cross(angvelB, rB);
    auto relvel = vA - vB;
    auto normal_relvel = dot(relvel, normal);

    auto [normal_row, normal_data] = cache.make_row();
    normal_data.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
    normal_data.lower_limit = 0;
    normal_row.restitution = cp.restitution;

    if (stiffness < large_scalar) {
        auto spring_force = cp.distance * stiffness;
        auto damper_force = normal_relvel * damping;
        normal_data.upper_limit = std::abs(spring_force + damper_force) * dt;
    } else {
        normal_data.upper_limit = large_scalar;
    }

    auto penetration = dot(posA + rA - posB - rB, normal);
    auto pvel = penetration / dt;

    normal_row.error = 0;

    // If not penetrating and the velocity necessary to touch in `dt` seconds
    // is smaller than the bounce velocity, it should apply an impulse that
    // will prevent penetration after the following physics update.
    if (penetration > 0 && pvel > -cp.restitution * normal_relvel) {
        normal_row.error = std::max(pvel, scalar(0));
    } else {
        // If this is a resting contact and it is penetrating, apply impulse to push it out.
        //if (cp.lifetime > 0) {
            normal_row.error = std::min(pvel, scalar(0));
        //}
    }
    
    auto tangent_relvel = relvel - normal * normal_relvel;
    auto tangent_relspd = length(tangent_relvel);
    auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

    auto [friction_row, friction_data] = cache.make_row();
    friction_data.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
    // friction_row limits are calculated in `iteration(...)` using the normal impulse.
    friction_data.lower_limit = friction_data.upper_limit = 0;
    friction_row.error = 0;

    m_friction = cp.friction;
}

void contact_constraint::iteration(entt::entity entity, const constraint &con, 
                                   entt::registry &registry, row_cache &cache,
                                   size_t row_index, scalar dt) {
    const auto &normal_data = cache.con_datas[row_index];
    auto &friction_data = cache.con_datas[row_index + 1];
    auto friction_impulse = std::abs(normal_data.impulse * m_friction);
    friction_data.lower_limit = -friction_impulse;
    friction_data.upper_limit = friction_impulse;
}

}
