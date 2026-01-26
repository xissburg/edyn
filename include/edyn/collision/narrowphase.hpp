#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/view.hpp>
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

class narrowphase {
    struct contact_point_construction_info {
        std::array<collision_result::collision_point, max_contacts> point;
        size_t count {0};
    };

    struct contact_point_destruction_info {
        std::vector<entt::entity> contact_entities;
    };

    void detect_collision_parallel();
    void detect_collision_parallel_range(unsigned start, unsigned end);
    void finish_detect_collision();

public:
    narrowphase(entt::registry &);

    void update(bool mt);

    /**
     * @brief Detects and processes collisions for the given manifolds.
     */
    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end);

private:
    entt::registry *m_registry;
    std::vector<contact_point_construction_info> m_cp_construction_infos;
    std::vector<contact_point_destruction_info> m_cp_destruction_infos;
    size_t m_max_sequential_size {4};
};

template<typename Iterator>
void narrowphase::update_contact_manifolds(Iterator begin, Iterator end) {
    auto &registry = *m_registry;
    auto manifold_view = registry.view<contact_manifold, contact_manifold_state>();
    auto dt = registry.ctx().get<settings>().fixed_dt;

    auto &async_settings = registry.ctx().get<const settings>().async_settings;
    const auto &transient = async_settings->contact_points_transient;

    for (auto it = begin; it != end; ++it) {
        entt::entity manifold_entity = *it;
        auto [manifold, manifold_state] = manifold_view.get(manifold_entity);
        collision_result result;
        detect_collision(registry, manifold.body, result);
        process_collision(registry, manifold_entity, result, dt,
                          [&, &manifold=manifold, &manifold_state=manifold_state](const collision_result::collision_point &rp) {
            create_contact_point(registry, manifold_entity, manifold, manifold_state, rp, transient);
        }, [&](entt::entity contact_entity) {
            destroy_contact_point(registry, contact_entity);
        });
    }
}

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP
