#include "edyn/collision/narrowphase.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/comp/material.hpp"

namespace edyn {

narrowphase::narrowphase(entt::registry &reg)
    : m_registry(&reg)
{}

bool narrowphase::parallelizable() const {
    return m_registry->size<contact_manifold>() > 1;
}

void narrowphase::update() {
    update_contact_distances(*m_registry);

    auto manifold_view = m_registry->view<contact_manifold>();
    update_contact_manifolds(manifold_view.begin(), manifold_view.end(), manifold_view);
}

void narrowphase::update_async(job &completion_job) {
    update_contact_distances(*m_registry);

    EDYN_ASSERT(parallelizable());

    auto manifold_view = m_registry->view<contact_manifold>();
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto cp_view = m_registry->view<contact_point>();
    auto imp_view = m_registry->view<constraint_impulse>();
    auto shapes_views_tuple = get_tuple_of_shape_views(*m_registry);

    // Resize result collection vectors to allocate one slot for each iteration
    // of the parallel_for.
    m_cp_construction_infos.resize(manifold_view.size());
    m_cp_destruction_infos.resize(manifold_view.size());
    auto &dispatcher = job_dispatcher::global();

    parallel_for_async(dispatcher, size_t{0}, manifold_view.size(), size_t{1}, completion_job,
            [this, body_view, tr_view, manifold_view, cp_view, imp_view, shapes_views_tuple] (size_t index) {
        auto entity = manifold_view[index];
        auto &manifold = manifold_view.get(entity);
        collision_result result;
        auto &construction_info = m_cp_construction_infos[index];
        auto &destruction_info = m_cp_destruction_infos[index];

        detect_collision(manifold.body, result, body_view, shapes_views_tuple);
        process_collision(entity, manifold, result, cp_view, imp_view, tr_view,
                          [&construction_info] (const collision_result::collision_point &rp) {
            construction_info.point[construction_info.count++] = rp;
        }, [&destruction_info] (entt::entity contact_entity) {
            destruction_info.contact_entity[destruction_info.count++] = contact_entity;
        });
    });
}

void narrowphase::finish_async_update() {
    auto manifold_view = m_registry->view<contact_manifold>();

    // Create contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &manifold = manifold_view.get(entity);
        auto &info_result = m_cp_construction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            auto contact_entity = create_contact_point(*m_registry, entity, manifold, info_result.point[j]);
            add_pending_contact(contact_entity, manifold.body);
        }
    }

    m_cp_construction_infos.clear();

    // Destroy contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &info_result = m_cp_destruction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            destroy_contact_point(*m_registry, entity, info_result.contact_entity[j]);
        }
    }

    m_cp_destruction_infos.clear();
}

void narrowphase::add_pending_contact(entt::entity contact_entity,
                                      std::array<entt::entity, 2> body) {
    if (m_registry->has<material>(body[0]) &&
        m_registry->has<material>(body[1])) {
        m_pending_contact_points.push_back(contact_entity);
    }
}

void narrowphase::create_contact_constraints() {
    auto cp_view = m_registry->view<contact_point>();

    for (auto contact_entity : m_pending_contact_points) {
        auto &cp = cp_view.get(contact_entity);
        create_contact_constraint(*m_registry, contact_entity, cp);
    }

    m_pending_contact_points.clear();
}

}
