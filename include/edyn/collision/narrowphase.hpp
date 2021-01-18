#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <memory>
#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/parallel/result_collector.hpp"

namespace edyn {

struct contact_manifold;
struct job;

using body_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, AABB, shape, position, orientation>; 
using transform_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, position, orientation>; 
using contact_manifold_collision_result_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, contact_manifold, collision_result>;

void detect_collision(contact_manifold &, collision_result &, const body_view_t &);
void process_result(entt::registry &, entt::entity manifold_entity, contact_manifold &, collision_result &, const transform_view_t &);

class narrowphase {
    struct contact_point_construction_info {
        entt::entity manifold_entity;
        collision_result::collision_point rp;
    };

    struct contact_point_destruction_info {
        entt::entity manifold_entity;
        entt::entity contact_entity;
    };

public:
    narrowphase(entt::registry &);

    bool parallelizable() const;
    void update();
    void update_async(job &completion_job);
    void finish_async_update();

    using contact_manifold_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, contact_manifold>;

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end) {
        auto manifold_result_view = m_registry->view<contact_manifold, collision_result>();
        update_contact_manifolds(begin, end, manifold_result_view);
    }

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end, contact_manifold_collision_result_view_t &manifold_result_view) {
        auto body_view = m_registry->view<AABB, shape, position, orientation>();
        auto tr_view = m_registry->view<position, orientation>();
        
        for (auto it = begin; it != end; ++it) {
            entt::entity entity = *it;
            auto [manifold, result] = manifold_result_view.get<contact_manifold, collision_result>(entity);
            detect_collision(manifold, result, body_view);
        }

        for (auto it = begin; it != end; ++it) {
            entt::entity entity = *it;
            auto [manifold, result] = manifold_result_view.get<contact_manifold, collision_result>(entity);
            process_result(*m_registry, entity, manifold, result, tr_view);
        }
    }

private:
    entt::registry *m_registry;
    std::unique_ptr<result_collector<contact_point_construction_info, 16>> m_cp_construction_results;
    std::unique_ptr<result_collector<contact_point_destruction_info, 16>> m_cp_destruction_results;
};

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP