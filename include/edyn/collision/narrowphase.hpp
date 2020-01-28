#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <entt/fwd.hpp>

namespace edyn {

struct relation;
struct contact_manifold;
struct collision_result;
struct vector3;
struct quaternion;

class narrowphase {
public:
    narrowphase(entt::registry &);
    void update();
    void on_construct_broadphase_relation(entt::entity, entt::registry &, relation &);

private:
    void process_collision(entt::entity, contact_manifold &, 
                           const relation &, const collision_result &);
    void prune(entt::entity, contact_manifold &,
               const vector3 &posA, const quaternion &ornA, 
               const vector3 &posB, const quaternion &ornB);

    entt::registry *registry;
};

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP