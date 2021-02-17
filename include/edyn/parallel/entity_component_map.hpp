#ifndef EDYN_PARALLEL_ENTITY_COMPONENT_MAP_HPP
#define EDYN_PARALLEL_ENTITY_COMPONENT_MAP_HPP

#include <unordered_map>
#include <unordered_set>
#include <entt/fwd.hpp>

namespace edyn {

struct entity_component_map_base {
    virtual ~entity_component_map_base() {}
    virtual bool empty() const = 0;
    virtual void clear() = 0;
};

template<typename Component>
struct entity_component_map: public entity_component_map_base {
    std::unordered_map<entt::entity, Component> map;

    void insert(entt::entity entity, const Component &comp) {
        map.insert_or_assign(entity, comp);
    }

    bool empty() const override {
        return map.empty();
    }

    void clear() override {
        map.clear();
    }
};

template<typename Component>
struct entity_component_set: public entity_component_map_base {
    std::unordered_set<entt::entity> set;

    void insert(entt::entity entity) {
        set.insert(entity);
    }

    bool empty() const override {
        return set.empty();
    }

    void clear() override {
        set.clear();
    }
};

}

#endif // EDYN_PARALLEL_ENTITY_COMPONENT_MAP_HPP
