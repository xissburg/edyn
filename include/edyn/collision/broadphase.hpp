#ifndef EDYN_COLLISION_BROADPHASE_HPP
#define EDYN_COLLISION_BROADPHASE_HPP

#include <entt/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include <map>
#include <utility>

namespace edyn {

struct relation;

class broadphase {
public:
    broadphase(entt::registry &);
    void update();

    using construct_relation_func_t = void(entt::entity, entt::registry &, relation &);
    using destroy_relation_func_t = void(entt::entity, entt::registry &);

    entt::sink<construct_relation_func_t> construct_relation_sink() {
        return {construct_relation_signal};
    }

    entt::sink<destroy_relation_func_t> destroy_relation_sink() {
        return {destroy_relation_signal};
    }

private:
    entt::registry *registry;
    // Maps pairs of entities to their relations.
    std::map<std::pair<entt::entity, entt::entity>, entt::entity> relations;
    entt::sigh<construct_relation_func_t> construct_relation_signal;
    entt::sigh<destroy_relation_func_t> destroy_relation_signal;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_HPP