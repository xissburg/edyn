#ifndef EDYN_UTIL_RIGIDBODY_HPP
#define EDYN_UTIL_RIGIDBODY_HPP

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct rigidbody_def {
    scalar mass;
    vector3 inertia;
    vector3 position {vector3_zero};
    quaternion orientation {quaternion_identity};
    vector3 linvel {vector3_zero};
    vector3 angvel {vector3_zero};
    bool presentation {false};
};

void make_rigidbody(entt::entity, entt::registry &, const rigidbody_def &);
entt::entity make_rigidbody(entt::registry &, const rigidbody_def &);

}

#endif // EDYN_UTIL_RIGIDBODY_HPP