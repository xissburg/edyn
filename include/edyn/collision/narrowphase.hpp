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
#include "edyn/constraints/constraint_impulse.hpp"
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
        std::array<entt::entity, max_contacts> contact_entity;
        size_t count {0};
    };

    void add_new_contact_point(entt::entity contact_entity,
                               std::array<entt::entity, 2> body);

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

    /**
     * @brief When new contact points are created, such as when
     * `update_contact_manifolds` is called, contact constraints are not created
     * immediately. This must be called to create contact constraints for new
     * contact points. It gives the caller control over when the constraints are
     * created.
     */
    void create_contact_constraints();

private:
    entt::registry *m_registry;
    std::vector<contact_point_construction_info> m_cp_construction_infos;
    std::vector<contact_point_destruction_info> m_cp_destruction_infos;
    std::vector<entt::entity> m_new_contact_points;
};

template<typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end) {
    auto manifold_view = m_registry->view<contact_manifold>();
    update_contact_manifolds(begin, end, manifold_view);
}

template<typename ContactManifoldView, typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end,
                                           ContactManifoldView &manifold_view) {
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto origin_view = m_registry->view<origin>();
    auto cp_view = m_registry->view<contact_point>();
    auto vel_view = m_registry->view<angvel>();
    auto rolling_view = m_registry->view<rolling_tag>();
    auto imp_view = m_registry->view<constraint_impulse>();
    auto tire_view = m_registry->view<tire_material>();
    auto views_tuple = get_tuple_of_shape_views(*m_registry);
    auto dt = m_registry->ctx<settings>().fixed_dt;

    for (auto it = begin; it != end; ++it) {
        entt::entity manifold_entity = *it;
        auto &manifold = manifold_view.get(manifold_entity);
        collision_result result;
        detect_collision(manifold.body, result, body_view, origin_view, views_tuple);

        process_collision(manifold_entity, manifold, result, cp_view, imp_view, tr_view, vel_view, rolling_view, origin_view, tire_view, dt,
                          [&] (const collision_result::collision_point &rp) {
            auto contact_entity = create_contact_point(*m_registry, manifold_entity, manifold, rp);
            add_new_contact_point(contact_entity, manifold.body);
        }, [&] (entt::entity contact_entity) {
            destroy_contact_point(*m_registry, manifold_entity, contact_entity);
        });
    }
}

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP
