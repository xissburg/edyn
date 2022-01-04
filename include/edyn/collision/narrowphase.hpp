#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/util/collision_util.hpp"
#include "edyn/context/settings.hpp"

namespace edyn {

struct job;

class narrowphase {
    struct contact_point_construction_info {
        std::array<collision_result::collision_point, max_contacts> point;
        size_t count {0};
    };

    struct contact_point_destruction_info {
        std::array<contact_manifold::contact_id_type, max_contacts> point_id;
        size_t count {0};
    };

    void clear_contact_manifold_events();

public:
    narrowphase(entt::registry &);

    bool parallelizable() const;
    void update();
    void update_async(job &completion_job);
    void finish_async_update();

    /**
     * @brief Detects and processes collisions for the given manifolds.
     */
    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end);

    template<typename ContactManifoldView, typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end,
                                  ContactManifoldView &manifold_view);

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

template<typename ContactManifoldView, typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end,
                                           ContactManifoldView &manifold_view) {
    auto events_view = m_registry->view<contact_manifold_events>();
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto origin_view = m_registry->view<origin>();
    auto vel_view = m_registry->view<angvel>();
    auto rolling_view = m_registry->view<rolling_tag>();
    auto material_view = m_registry->view<material>();
    auto orn_view = m_registry->view<orientation>();
    auto mesh_shape_view = m_registry->view<mesh_shape>();
    auto paged_mesh_shape_view = m_registry->view<paged_mesh_shape>();
    auto views_tuple = get_tuple_of_shape_views(*m_registry);
    auto dt = m_registry->ctx<settings>().fixed_dt;

    for (auto it = begin; it != end; ++it) {
        entt::entity manifold_entity = *it;
        auto &manifold = manifold_view.template get<contact_manifold>(manifold_entity);
        auto &events = events_view.get<contact_manifold_events>(manifold_entity);
        collision_result result;
        detect_collision(manifold.body, result, body_view, origin_view, views_tuple);

        process_collision(manifold_entity, manifold, events, result, tr_view, vel_view,
                          rolling_view, origin_view, orn_view, material_view,
                          mesh_shape_view, paged_mesh_shape_view, dt,
                          [&] (const collision_result::collision_point &rp) {
            create_contact_point(*m_registry, manifold_entity, manifold, rp);
        }, [&] (auto pt_id) {
            destroy_contact_point(*m_registry, manifold_entity, pt_id);
        });
    }
}

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP
