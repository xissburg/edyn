#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <entt/fwd.hpp>
#include <entt/signal/sigh.hpp>

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
    void on_destroy_broadphase_relation(entt::entity, entt::registry &);

    using contact_func_t = void(entt::entity, entt::registry &, const relation &, 
                                contact_manifold &, size_t);

    entt::sink<contact_func_t> contact_started_sink() {
        return {contact_started_signal};
    }

    entt::sink<contact_func_t> contact_ended_sink() {
        return {contact_ended_signal};
    }

private:
    void process_collision(entt::entity, contact_manifold &, 
                           const relation &, const collision_result &);
    void prune(entt::entity, contact_manifold &, const relation &rel,
               const vector3 &posA, const quaternion &ornA, 
               const vector3 &posB, const quaternion &ornB);

    entt::registry *registry;
    entt::sigh<contact_func_t> contact_started_signal;
    entt::sigh<contact_func_t> contact_ended_signal;
};

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP