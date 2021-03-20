#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include <entt/entt.hpp>

namespace edyn {

contact_manifold_map::contact_manifold_map(entt::registry &registry) {
    registry.on_construct<contact_manifold>().connect<&contact_manifold_map::on_construct_contact_manifold>(*this);
    registry.on_destroy<contact_manifold>().connect<&contact_manifold_map::on_destroy_contact_manifold>(*this);
}

bool contact_manifold_map::contains(const entity_pair &pair) const {
    return m_pair_map.count(pair) > 0;
}

bool contact_manifold_map::contains(entt::entity first, entt::entity second) const {
    return contains(std::make_pair(first, second));
}

void contact_manifold_map::on_construct_contact_manifold(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<contact_manifold>(entity);
    // Insert all permutations.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    EDYN_ASSERT(m_pair_map.count(p) == 0 && m_pair_map.count(q) == 0);
    m_pair_map[p] = entity;
    m_pair_map[q] = entity;
}

void contact_manifold_map::on_destroy_contact_manifold(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<contact_manifold>(entity);
    // Cleanup cached info.
    auto p = std::make_pair(manifold.body[0], manifold.body[1]);
    auto q = std::make_pair(manifold.body[1], manifold.body[0]);
    m_pair_map.erase(p);
    m_pair_map.erase(q);
}

}
