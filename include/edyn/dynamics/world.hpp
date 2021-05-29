#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/parallel/island_coordinator.hpp"

namespace edyn {

class world;

class world final {
public:
    world(entt::registry &);
    ~world();

    void update();

    void set_paused(bool);
    void step();

    /**
     * @brief Updates components in the islands where the entity resides.
     */
    template<typename... Component>
    void refresh(entt::entity entity) {
        m_island_coordinator.refresh<Component...>(entity);
    }

    void set_fixed_dt(scalar dt);

    auto get_fixed_dt() {
        return m_fixed_dt;
    }

private:
    entt::registry* m_registry;
    broadphase_main m_bphase;
    island_coordinator m_island_coordinator;

    scalar m_fixed_dt {scalar(1.0/60)};
    bool m_paused {false};
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP
