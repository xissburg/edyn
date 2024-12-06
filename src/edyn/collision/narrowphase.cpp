#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/context/task.hpp"
#include "edyn/context/task_util.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/signal/delegate.hpp>

namespace edyn {

narrowphase::narrowphase(entt::registry &reg)
    : m_registry(&reg)
{}

void narrowphase::clear_contact_manifold_events() {
    m_registry->view<contact_manifold_events>().each([](auto &events) {
        events = {};
    });
}

void narrowphase::update(bool mt) {
    clear_contact_manifold_events();
    update_contact_distances(*m_registry);

    auto manifold_view = m_registry->view<contact_manifold>(exclude_sleeping_disabled);
    auto num_active_manifolds = calculate_view_size(manifold_view);

    if (mt && num_active_manifolds > m_max_sequential_size) {
        detect_collision_parallel();
        finish_detect_collision();
    } else {
        update_contact_manifolds(manifold_view.begin(), manifold_view.end());
    }
}

void narrowphase::detect_collision_parallel_range(unsigned start, unsigned end) {
    auto &registry = *m_registry;
    auto manifold_view = registry.view<contact_manifold>();
    auto events_view = registry.view<contact_manifold_events>();
    auto body_view = registry.view<AABB, shape_index, position, orientation>();
    auto tr_view = registry.view<position, orientation>();
    auto vel_view = registry.view<angvel>();
    auto rolling_view = registry.view<rolling_tag>();
    auto origin_view = registry.view<origin>();
    auto material_view = registry.view<material>();
    auto orn_view = registry.view<orientation>();
    auto mesh_shape_view = registry.view<mesh_shape>();
    auto paged_mesh_shape_view = registry.view<paged_mesh_shape>();
    auto shapes_views_tuple = get_tuple_of_shape_views(registry);
    auto dt = registry.ctx().get<settings>().fixed_dt;
    auto first = manifold_view.begin();
    std::advance(first, start);

    for (auto index = start; index < end; ++index, ++first) {
        auto entity = *first;
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
    }
}

void narrowphase::detect_collision_parallel() {
    // Resize result collection vectors to allocate one slot for each iteration.
    auto manifold_view = m_registry->view<contact_manifold>();
    m_cp_construction_infos.resize(manifold_view.size());
    m_cp_destruction_infos.resize(manifold_view.size());

    auto task = task_delegate_t(entt::connect_arg_t<&narrowphase::detect_collision_parallel_range>{}, *this);
    enqueue_task_wait(*m_registry, task, manifold_view.size());
}

void narrowphase::finish_detect_collision() {
    auto manifold_view = m_registry->view<contact_manifold>();
    auto it = manifold_view.begin();

    // Destroy contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i, ++it) {
        auto entity = *it;
        auto &info_result = m_cp_destruction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            destroy_contact_point(*m_registry, entity, info_result.point_id[j]);
        }
    }

    // Create contact points.
    it = manifold_view.begin();

    for (size_t i = 0; i < manifold_view.size(); ++i, ++it) {
        auto entity = *it;
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
