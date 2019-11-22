#include <entt/entt.hpp>
#include "edyn/util/rigidbody.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/present_orientation.hpp"

namespace edyn {

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
    
    registry.assign<edyn::position>(entity, def.position);
    registry.assign<edyn::orientation>(entity, def.orientation);

    if (def.kind == rigidbody_kind::rb_dynamic) {
        registry.assign<edyn::mass>(entity, def.mass);
        registry.assign<edyn::inertia>(entity, def.inertia);
    } else {
        registry.assign<edyn::mass>(entity, EDYN_SCALAR_MAX);
        registry.assign<edyn::inertia>(entity, vector3_max);
    }

    if (def.kind == rigidbody_kind::rb_static) {
        registry.assign<edyn::linvel>(entity, vector3_zero);
        registry.assign<edyn::angvel>(entity, vector3_zero);
    } else {
        registry.assign<edyn::linvel>(entity, def.linvel);
        registry.assign<edyn::angvel>(entity, def.angvel);
    }

    if (def.presentation) {
        registry.assign<edyn::present_position>(entity);
        registry.assign<edyn::present_orientation>(entity);
    }
}

entt::entity make_rigidbody(entt::registry &registry, const rigidbody_def &def) {
    auto ent = registry.create();
    make_rigidbody(ent, registry, def);
    return ent;
}

void update_kinematic_position(entt::registry &registry, entt::entity entity, const vector3 &pos, scalar dt) {
    auto &curpos = registry.get<position>(entity);
    auto &vel = registry.get<linvel>(entity);
    vel = (pos - curpos) / dt;
    curpos = pos;
}

}
