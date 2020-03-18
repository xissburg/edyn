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

    entt::sink<construct_relation_func_t> construct_relation_sink() {
        return {construct_relation_signal};
    }

    /**
     * Iterates over every broadphase relation of a certain entity. Useful to
     * process all contacts of an entity.
     */ 
    template<typename Func>
    void each_relation(entt::entity entity, Func f) {
        for (auto& pair : relations) {
            if (pair.first.first == entity || pair.first.second == entity) {
                f(pair.second);
            }
        }
    }

private:
    entt::registry *registry;
    // Maps pairs of entities to their relations.
    std::map<std::pair<entt::entity, entt::entity>, entt::entity> relations;
    entt::sigh<construct_relation_func_t> construct_relation_signal;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_HPP