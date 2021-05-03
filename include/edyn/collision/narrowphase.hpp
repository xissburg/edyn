#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <array>
#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/collision_result.hpp"

namespace edyn {

struct contact_manifold;
struct job;

using body_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, AABB, shape_index, position, orientation>; 
using transform_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, position, orientation>; 
using contact_manifold_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, contact_manifold>;

void detect_collision(const contact_manifold &, collision_result &, 
                      const body_view_t &, const shapes_view_t &);
void process_result(entt::registry &, entt::entity manifold_entity, 
                    contact_manifold &, const collision_result &, 
                    const transform_view_t &);

class narrowphase {
    struct contact_point_construction_info {
        std::array<collision_result::collision_point, max_contacts> point;
        size_t count {0};
    };

    struct contact_point_destruction_info {
        std::array<entt::entity, max_contacts> contact_entity;
        size_t count {0};
    };

public:
    narrowphase(entt::registry &);

    bool parallelizable() const;
    void update();
    void update_async(job &completion_job);
    void finish_async_update();

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end);

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end, 
                                  contact_manifold_view_t &manifold_view);

private:
    entt::registry *m_registry;
    std::vector<contact_point_construction_info> m_cp_construction_infos;
    std::vector<contact_point_destruction_info> m_cp_destruction_infos;
};

template<typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end) {
    auto manifold_view = m_registry->view<contact_manifold>();
    update_contact_manifolds(begin, end, manifold_view);
}

template<typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end, 
                                           contact_manifold_view_t &manifold_view) {
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto shapes_view = get_shapes_view(*m_registry);

    for (auto it = begin; it != end; ++it) {
        entt::entity entity = *it;
        auto &manifold = manifold_view.get(entity);
        collision_result result;
        detect_collision(manifold, result, body_view, shapes_view);
        process_result(*m_registry, entity, manifold, result, tr_view);
    }
}

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP
