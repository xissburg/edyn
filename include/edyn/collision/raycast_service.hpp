#ifndef EDYN_COLLISION_RAYCAST_SERVICE_HPP
#define EDYN_COLLISION_RAYCAST_SERVICE_HPP

#include "edyn/collision/raycast.hpp"
#include "edyn/math/vector3.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

class raycast_service {
    enum class state {
        begin,
        broadphase,
        narrowphase,
        done
    };

    struct broadphase_context {
        unsigned id;
        vector3 p0, p1;
        std::vector<entt::entity> candidates;
    };

    struct narrowphase_context {
        unsigned id;
        vector3 p0, p1;
        entt::entity entity;
        shape_raycast_result result;
    };

    bool run_state_machine(job &completion_job);
    bool run_broadphase(job &completion_job);
    bool run_narrowphase(job &completion_job);
    void finish_broadphase();
    void finish_narrowphase();

public:
    raycast_service(entt::registry &registry);

    void add_ray(vector3 p0, vector3 p1, unsigned id) {
        m_broad_ctx.push_back(broadphase_context{id, p0, p1});
    }

    bool update(job &completion_job);

    template<typename Func>
    void consume_results(Func func) {
        for (auto [id, res] : m_results) {
            func(id, res);
        }
        m_results.clear();
    }

private:
    entt::registry *m_registry;
    state m_state {state::begin};

    std::vector<broadphase_context> m_broad_ctx;
    std::vector<narrowphase_context> m_narrow_ctx;
    std::unordered_map<unsigned, raycast_result> m_results;

    size_t m_max_raycast_broadphase_sequential_size {4};
    size_t m_max_raycast_narrowphase_sequential_size {4};
};

}

#endif // EDYN_COLLISION_RAYCAST_SERVICE_HPP
