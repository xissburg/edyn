#ifndef EDYN_UTIL_ENTITY_MAP_HPP
#define EDYN_UTIL_ENTITY_MAP_HPP

#include <map>
#include <entt/entity/fwd.hpp>

namespace edyn {

class entity_map {
public:
    void insert(entt::entity entity, entt::entity other) {
        map[entity] = other;
        others[other] = entity;
    }

    void erase(entt::entity entity) {
        auto other = map.at(entity);
        map.erase(entity);
        others.erase(other);
    }

    void erase_other(entt::entity other) {
        auto entity = others.at(other);
        map.erase(entity);
        others.erase(other);
    }

    bool contains(entt::entity entity) const {
        return map.count(entity) > 0;
    }

    bool contains_other(entt::entity other) const {
        return others.count(other) > 0;
    }

    entt::entity at(entt::entity entity) const {
        return map.at(entity);
    }

    entt::entity at_other(entt::entity other) const {
        return others.at(other);
    }

    template<typename Predicate>
    void erase_if(Predicate predicate) {
        for (auto it = map.begin(); it != map.end();) {
            if (predicate(it->first, it->second)) {
                others.erase(it->second);
                it = map.erase(it);
            } else {
                ++it;
            }
        }
    }

    template<typename Func>
    void each(Func func) const {
        for (auto it = map.begin(); it != map.end(); ++it) {
            func(it->first, it->second);
        }
    }

    void swap() {
        std::swap(map, others);
    }

private:
    std::map<entt::entity, entt::entity> map;
    std::map<entt::entity, entt::entity> others;
};

}

#endif // EDYN_UTIL_ENTITY_MAP_HPP
