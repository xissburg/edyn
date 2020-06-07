#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <entt/fwd.hpp>
#include <entt/signal/sigh.hpp>

namespace edyn {

struct relation;
struct contact_point;
struct contact_manifold;
struct collision_result;
struct vector3;
struct quaternion;

class narrowphase {
public:
    narrowphase(entt::registry &);
    void update();

private:
    void process_collision(entt::entity, contact_manifold &, 
                           relation &, const collision_result &);
    void prune(entt::entity, contact_manifold &, const relation &rel,
               const vector3 &posA, const quaternion &ornA, 
               const vector3 &posB, const quaternion &ornB);

    entt::registry *registry;
    std::vector<entt::scoped_connection> connections;
};

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP