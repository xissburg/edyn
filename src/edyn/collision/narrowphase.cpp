#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"

namespace edyn {

narrowphase::narrowphase(entt::registry &reg)
    : m_registry(&reg)
{}

void narrowphase::clear_contact_manifold_events() {
    m_registry->view<contact_manifold_events>().each([](auto &events) {
        events = {};
    });
}

void narrowphase::update_sequential(bool mt) {
    clear_contact_manifold_events();
    update_contact_distances(*m_registry);

    auto manifold_view = m_registry->view<contact_manifold>(exclude_sleeping_disabled);
    auto num_active_manifolds = calculate_view_size(manifold_view);

    if (mt && num_active_manifolds > m_max_sequential_size) {
        detect_collision_parallel(false);
        finish_detect_collision();
    } else {
        update_contact_manifolds(manifold_view.begin(), manifold_view.end());
    }
}

bool narrowphase::update(const job &completion_job) {
    switch (m_state) {
    case state::begin: {
        clear_contact_manifold_events();
        update_contact_distances(*m_registry);
        auto manifold_view = m_registry->view<contact_manifold>(exclude_sleeping_disabled);
        auto num_active_manifolds = calculate_view_size(manifold_view);

        if (num_active_manifolds <= m_max_sequential_size) {
            auto manifold_view = m_registry->view<contact_manifold>();
            update_contact_manifolds(manifold_view.begin(), manifold_view.end());
            return true;
        } else {
            m_state = state::detect_collision;
            detect_collision_parallel(true, completion_job);
            return false;
        }
        break;
    }
    case state::detect_collision:
        finish_detect_collision();
        m_state = state::begin;
        return true;
    }
}

void narrowphase::detect_collision_parallel(bool async, const job &completion_job) {
    auto manifold_view = m_registry->view<contact_manifold>();
    auto events_view = m_registry->view<contact_manifold_events>();
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto vel_view = m_registry->view<angvel>();
    auto rolling_view = m_registry->view<rolling_tag>();
    auto origin_view = m_registry->view<origin>();
    auto material_view = m_registry->view<material>();
    auto orn_view = m_registry->view<orientation>();
    auto mesh_shape_view = m_registry->view<mesh_shape>();
    auto paged_mesh_shape_view = m_registry->view<paged_mesh_shape>();
    auto shapes_views_tuple = get_tuple_of_shape_views(*m_registry);
    auto dt = m_registry->ctx().at<settings>().fixed_dt;

    // Resize result collection vectors to allocate one slot for each iteration
    // of the parallel_for.
    m_cp_construction_infos.resize(manifold_view.size());
    m_cp_destruction_infos.resize(manifold_view.size());
    auto &dispatcher = job_dispatcher::global();

    auto for_loop_body = [this, body_view, tr_view, vel_view, rolling_view, origin_view,
             manifold_view, events_view, orn_view, material_view, mesh_shape_view,
             paged_mesh_shape_view, shapes_views_tuple, dt](size_t index) {
        auto entity = manifold_view[index];
        auto [manifold] = manifold_view.get(entity);
        auto [events] = events_view.get(entity);
        collision_result result;
        auto &construction_info = m_cp_construction_infos[index];
        auto &destruction_info = m_cp_destruction_infos[index];

        detect_collision(manifold.body, result, body_view, origin_view, shapes_views_tuple);
        process_collision(entity, manifold, events, result, tr_view, vel_view,
                          rolling_view, origin_view, orn_view, material_view,
                          mesh_shape_view, paged_mesh_shape_view, dt,
                          [&construction_info](const collision_result::collision_point &rp) {
            construction_info.point[construction_info.count++] = rp;
        }, [&destruction_info](auto pt_id) {
            EDYN_ASSERT(pt_id < max_contacts);
            destruction_info.point_id[destruction_info.count++] = pt_id;
        });
    };

    if (async) {
        parallel_for_async(dispatcher, size_t{0}, manifold_view.size(), size_t{1}, completion_job, for_loop_body);
    } else {
        parallel_for(dispatcher, size_t{}, manifold_view.size(), size_t{1}, for_loop_body);
    }
}

void narrowphase::finish_detect_collision() {
    auto manifold_view = m_registry->view<contact_manifold>();

    // Destroy contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &info_result = m_cp_destruction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            destroy_contact_point(*m_registry, entity, info_result.point_id[j]);
        }
    }

    // Create contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &manifold = manifold_view.get<contact_manifold>(entity);
        auto &info_result = m_cp_construction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            create_contact_point(*m_registry, entity, manifold, info_result.point[j]);
        }
    }

    m_cp_destruction_infos.clear();
    m_cp_construction_infos.clear();
}

}
