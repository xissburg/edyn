#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/context/task.hpp"
#include "edyn/context/task_util.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/util/collision_util.hpp"
#include "edyn/util/contact_manifold_util.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/signal/delegate.hpp>

namespace edyn {

narrowphase::narrowphase(entt::registry &reg)
    : m_registry(&reg)
{
}

void narrowphase::update(bool mt) {
    for (auto entity : m_registry->view<clear_contact_manifold_tag>()) {
        clear_contact_manifold(*m_registry, entity);
    }
    m_registry->clear<clear_contact_manifold_tag>();

    m_new_contacts.clear();

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
    auto dt = registry.ctx().get<settings>().fixed_dt;
    auto manifold_view = registry.view<contact_manifold>();
    auto first = manifold_view.begin();
    std::advance(first, start);

    for (auto index = start; index < end; ++index, ++first) {
        auto entity = *first;
        auto [manifold] = manifold_view.get(entity);
        collision_result result;
        auto &construction_info = m_cp_construction_infos[index];
        auto &destruction_info = m_cp_destruction_infos[index];

        detect_collision(registry, manifold.body, result);
        process_collision(registry, entity, result, dt,
            [&construction_info](const collision_result::collision_point &rp) {
                construction_info.point[construction_info.count++] = rp;
            }, [&destruction_info](entt::entity contact_entity) {
                destruction_info.contact_entities.push_back(contact_entity);
            });
    }
}

void narrowphase::detect_collision_parallel() {
    // Resize result collection vectors to allocate one slot for each iteration.
    auto num_manifolds = m_registry->view<contact_manifold>().size();
    m_cp_construction_infos.resize(num_manifolds);
    m_cp_destruction_infos.resize(num_manifolds);

    auto task = task_delegate_t(entt::connect_arg_t<&narrowphase::detect_collision_parallel_range>{}, *this);
    enqueue_task_wait(*m_registry, task, static_cast<unsigned int>(num_manifolds));
}

void narrowphase::finish_detect_collision() {
    auto num_manifolds = m_registry->view<contact_manifold>().size();
    auto manifold_view = m_registry->view<contact_manifold, contact_manifold_state>();
    auto it = manifold_view.begin();

    // Destroy contact points.
    for (size_t i = 0; i < num_manifolds; ++i, ++it) {
        auto &info_result = m_cp_destruction_infos[i];

        for (auto contact_entity : info_result.contact_entities) {
            destroy_contact_point(*m_registry, contact_entity);
        }
    }

    // Create contact points.
    it = manifold_view.begin();

    auto &async_settings = m_registry->ctx().get<const settings>().async_settings;
    const auto &transient = async_settings->contact_points_transient;

    for (size_t i = 0; i < num_manifolds; ++i, ++it) {
        auto entity = *it;
        auto [manifold, manifold_state] = manifold_view.get(entity);
        auto &info_result = m_cp_construction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            auto contact_entity = create_contact_point(*m_registry, entity, manifold, manifold_state, info_result.point[j], transient);
            m_new_contacts.push_back(contact_entity);
        }
    }

    m_cp_destruction_infos.clear();
    m_cp_construction_infos.clear();
}

void narrowphase::patch_new_contacts() {
    auto &registry = *m_registry;

    for (auto contact_entity : m_new_contacts) {
        // Patch components that might have changed after solving constraints.
        if (registry.all_of<contact_point_impulse>(contact_entity)) {
            registry.patch<contact_point_impulse>(contact_entity);
        }
        if (registry.all_of<contact_point_spin_friction_impulse>(contact_entity)) {
            registry.patch<contact_point_spin_friction_impulse>(contact_entity);
        }
        if (registry.all_of<contact_point_roll_friction_impulse>(contact_entity)) {
            registry.patch<contact_point_roll_friction_impulse>(contact_entity);
        }

        registry.emplace<contact_started_tag>(contact_entity);
    }

    m_new_contacts.clear();
}

}
