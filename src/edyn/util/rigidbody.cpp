#include <entt/entt.hpp>
#include "edyn/util/rigidbody.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/present_position.hpp"

namespace edyn {

void make_rigidbody(entt::entity entity, entt::registry &registry, const rigidbody_def &def) {
    registry.assign<edyn::position>(entity, def.position);
    registry.assign<edyn::linvel>(entity, def.linvel);
    registry.assign<edyn::angvel>(entity, def.angvel);
    registry.assign<edyn::mass>(entity, def.mass);
    registry.assign<edyn::inertia>(entity, def.inertia);

    if (def.presentation) {
        registry.assign<edyn::present_position>(entity);
        //registry.assign<edyn::present_orientation>(entity);
    }
}

entt::entity make_rigidbody(entt::registry &registry, const rigidbody_def &def) {
    auto ent = registry.create();
    make_rigidbody(ent, registry, def);
    return ent;
}

}