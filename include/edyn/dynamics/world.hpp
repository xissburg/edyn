#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <entt/fwd.hpp>

#include "solver.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/parallel/registry_delta.hpp"

namespace edyn {

class world;

class world final {
public:
    world(entt::registry &);
    ~world();

    void update();

    void set_paused(bool);
    void step();

    template<typename... Component>
    void refresh(entt::entity entity) {
        static_assert(sizeof...(Component) > 0);
        auto &dirty = m_registry->get_or_emplace<island_node_dirty>(entity);
        (dirty.updated_indexes.insert(entt::type_index<Component>::value()), ...);
    }

    void on_broadphase_intersect(entt::entity, entt::entity);

    scalar m_fixed_dt {1.0/60};
    solver m_solver;

private:
    entt::registry* m_registry;
    broadphase_main m_bphase;
    island_coordinator m_island_coordinator;
    std::vector<entt::scoped_connection> m_connections;

    bool m_paused {false};
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP