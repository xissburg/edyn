#include <entt/entt.hpp>
#include "edyn/util/rigidbody.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/linacc.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/present_orientation.hpp"

namespace edyn {

void rigidbody_def::update_inertia() {
    std::visit([&] (auto &&s) {
        inertia = s.inertia(mass);
    }, *shape_opt);
}

void make_rigidbody(entt::entity entity, entt::registry &registry, const rigidbody_def &def) {
    switch (def.kind) {
    case rigidbody_kind::rb_dynamic:
        registry.assign<dynamic_tag>(entity);
        break;
    case rigidbody_kind::rb_kinematic:
        registry.assign<kinematic_tag>(entity);
        break;
    case rigidbody_kind::rb_static:
        registry.assign<static_tag>(entity);
        break;
    }
    
    registry.assign<position>(entity, def.position);
    registry.assign<orientation>(entity, def.orientation);

    if (def.kind == rigidbody_kind::rb_dynamic) {
        registry.assign<mass>(entity, def.mass);
        registry.assign<inertia>(entity, def.inertia);

        if (def.spins) {
            registry.assign<spin_angle>(entity, 0);
            registry.assign<spin>(entity, 0);
        }
    } else {
        registry.assign<mass>(entity, EDYN_SCALAR_MAX);
        registry.assign<inertia>(entity, vector3_max);
    }

    if (def.kind == rigidbody_kind::rb_static) {
        registry.assign<linvel>(entity, vector3_zero);
        registry.assign<angvel>(entity, vector3_zero);
    } else {
        registry.assign<linvel>(entity, def.linvel);
        registry.assign<angvel>(entity, def.angvel);
    }

    if (def.kind == rigidbody_kind::rb_dynamic && def.gravity != vector3_zero) {
        registry.assign<linacc>(entity, def.gravity);
    }

    if (!def.sensor) {
        registry.assign<material>(entity, def.restitution, def.friction,
                                  def.stiffness, def.damping, def.use_contact_patch, def.is_tire);
    }

    if (def.presentation) {
        registry.assign<present_position>(entity, def.position);
        registry.assign<present_orientation>(entity, def.orientation);
    }

    if (auto opt = def.shape_opt) {
        registry.assign<shape>(entity, *opt);
    }
}

entt::entity make_rigidbody(entt::registry &registry, const rigidbody_def &def) {
    auto ent = registry.create();
    make_rigidbody(ent, registry, def);
    return ent;
}

void update_kinematic_position(entt::registry &registry, entt::entity entity, const vector3 &pos, scalar dt) {
    EDYN_ASSERT(registry.has<kinematic_tag>(entity));
    auto &curpos = registry.get<position>(entity);
    auto &vel = registry.get<linvel>(entity);
    vel = (pos - curpos) / dt;
    curpos = pos;
}

void update_kinematic_orientation(entt::registry &registry, entt::entity entity, const quaternion &orn, scalar dt) {
    EDYN_ASSERT(registry.has<kinematic_tag>(entity));
    auto &curorn = registry.get<orientation>(entity);
    auto q = normalize(orn * conjugate(curorn));
    auto &vel = registry.get<angvel>(entity);
    vel = (quaternion_axis(q) * quaternion_angle(q)) / dt;
    curorn = orn;
}

void clear_kinematic_velocities(entt::registry &registry) {
    auto view = registry.view<kinematic_tag, linvel, angvel>();
    view.each([] ([[maybe_unused]] auto, linvel &v, angvel &w) {
        v = vector3_zero;
        w = vector3_zero;
    });

    registry.view<kinematic_tag, spin>().each([] ([[maybe_unused]] auto, spin &s) {
        s.s = 0;
    });
}

}
