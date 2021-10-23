#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
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
    auto vel_view = m_registry->view<angvel>();
    auto rolling_view = m_registry->view<rolling_tag>();
    auto origin_view = m_registry->view<origin>();
    auto cp_view = m_registry->view<contact_point>();
    auto imp_view = m_registry->view<constraint_impulse>();
    auto material_view = m_registry->view<material>();
    auto mesh_shape_view = m_registry->view<mesh_shape>();
    auto paged_mesh_shape_view = m_registry->view<paged_mesh_shape>();
    auto shapes_views_tuple = get_tuple_of_shape_views(*m_registry);
    auto dt = m_registry->ctx<settings>().fixed_dt;

    // Resize result collection vectors to allocate one slot for each iteration
    // of the parallel_for.
    m_cp_construction_infos.resize(manifold_view.size());
    m_cp_destruction_infos.resize(manifold_view.size());
    auto &dispatcher = job_dispatcher::global();

    parallel_for_async(dispatcher, size_t{0}, manifold_view.size(), size_t{1}, completion_job,
            [this, body_view, tr_view, vel_view, rolling_view, origin_view,
             manifold_view, cp_view, imp_view, material_view, mesh_shape_view,
             paged_mesh_shape_view, shapes_views_tuple, dt] (size_t index) {
        auto entity = manifold_view[index];
        auto &manifold = manifold_view.get<contact_manifold>(entity);
        collision_result result;
        auto &construction_info = m_cp_construction_infos[index];
        auto &destruction_info = m_cp_destruction_infos[index];

        detect_collision(manifold.body, result, body_view, origin_view, shapes_views_tuple);
        process_collision(entity, manifold, result, cp_view, imp_view, tr_view,
                          vel_view, rolling_view, origin_view, material_view,
                          mesh_shape_view, paged_mesh_shape_view, dt,
                          [&construction_info] (const collision_result::collision_point &rp) {
            construction_info.point[construction_info.count++] = rp;
        }, [&destruction_info] (entt::entity contact_entity) {
            EDYN_ASSERT(contact_entity != entt::null);
            destruction_info.contact_entity[destruction_info.count++] = contact_entity;
        });
    });
}

void narrowphase::finish_async_update() {
    auto manifold_view = m_registry->view<contact_manifold>();

    // Destroy contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &info_result = m_cp_destruction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            destroy_contact_point(*m_registry, entity, info_result.contact_entity[j]);
        }
    }

    // Create contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &manifold = manifold_view.get<contact_manifold>(entity);
        auto &info_result = m_cp_construction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            auto contact_entity = create_contact_point(*m_registry, entity, manifold, info_result.point[j]);
            add_new_contact_point(contact_entity, manifold.body);
        }
    }

    m_cp_destruction_infos.clear();
    m_cp_construction_infos.clear();
}

void narrowphase::add_new_contact_point(entt::entity contact_entity,
                                        std::array<entt::entity, 2> body) {
    if (m_registry->any_of<material>(body[0]) &&
        m_registry->any_of<material>(body[1])) {
        m_new_contact_points.push_back(contact_entity);
    }
}

void narrowphase::create_contact_constraints() {
    auto cp_view = m_registry->view<contact_point>();
    auto mat_view = m_registry->view<material>();

    for (auto contact_entity : m_new_contact_points) {
        if (!m_registry->valid(contact_entity)) {
            continue; // Might've been destroyed.
        }

        auto &cp = cp_view.get<contact_point>(contact_entity);

        if (mat_view.contains(cp.body[0]) && mat_view.contains(cp.body[1])) {
            create_contact_constraint(*m_registry, contact_entity, cp);
        }
    }

    m_new_contact_points.clear();
}

}
