#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/dynamics/solver_stage.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

struct constraint;
struct relation;
struct collision_result;

struct contact_constraint {
    contact_manifold manifold {};

    template<solver_stage Stage>
    void update(constraint &con, const relation &rel, entt::registry &reg, scalar dt) {
        if constexpr(Stage == solver_stage::init) {
            init(con, rel, reg);
        }
    }

    void init(constraint &, const relation &, entt::registry &);
    void prepare(constraint &, const relation &, entt::registry &, scalar dt);
    void before_solve(constraint &, const relation &, entt::registry &, scalar dt);

private:
    void process_collision(const collision_result &, constraint &, const relation &, entt::registry &);
    void prune(const vector3 &posA, const quaternion &ornA, const vector3 &posB, const quaternion &ornB, constraint &, entt::registry &);
    void setup_rows(const vector3 &posA, const quaternion &ornA, const vector3 &posB, const quaternion &ornB, const relation &rel, entt::registry &registry, scalar dt);
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP