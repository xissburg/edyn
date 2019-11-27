#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

struct collision_result;

struct contact_constraint : public constraint_base<contact_constraint> {
    contact_manifold manifold {};

    void prepare(constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(constraint &, const relation &, entt::registry &, scalar dt);

private:
    void process_collision(const collision_result &, constraint &, const relation &, entt::registry &);
    void prune(const vector3 &posA, const quaternion &ornA, 
               const vector3 &posB, const quaternion &ornB, 
               constraint &, entt::registry &);
    void setup_rows(const vector3 &posA, const quaternion &ornA, 
                    const vector3 &posB, const quaternion &ornB, 
                    const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP