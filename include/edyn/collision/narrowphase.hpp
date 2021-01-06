#ifndef EDYN_COLLISION_NARROWPHASE_HPP
#define EDYN_COLLISION_NARROWPHASE_HPP

#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

struct contact_manifold;
struct job;

class narrowphase {
public:
    narrowphase(entt::registry &);

    bool parallelizable() const;
    void update();
    void update_async(job &completion_job);
    void finish_async_update();

    using body_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, AABB, shape, position, orientation>; 
    using contact_manifold_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, contact_manifold>;

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end) {
        auto manifold_view = m_registry->view<contact_manifold>();
        update_contact_manifolds(begin, end, manifold_view);
    }

    template<typename Iterator>
    void update_contact_manifolds(Iterator begin, Iterator end, contact_manifold_view_t &manifold_view) {
        auto body_view = m_registry->view<AABB, shape, position, orientation>();
        
        for (auto it = begin; it != end; ++it) {
            entt::entity entity = *it;
            auto &manifold = manifold_view.get(entity);
            update_contact_manifold(entity, manifold, body_view);
        }
    }

    void update_contact_manifold(entt::entity, contact_manifold &, narrowphase::body_view_t &);

private:
    entt::registry *m_registry;
};

}

#endif // EDYN_COLLISION_NARROWPHASE_HPP