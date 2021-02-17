#ifndef EDYN_COLLISION_BROADPHASE_MAIN_HPP
#define EDYN_COLLISION_BROADPHASE_MAIN_HPP

#include <map>
#include <vector>
#include <entt/fwd.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/util/entity_pair.hpp"

namespace edyn {

class tree_view;

class broadphase_main {

    // A higher threshold is used in the main broadphase to create contact 
    // manifolds between different islands a little earlier and decrease the
    // probability they'll arrive in the corresponding island worker when the
    // shapes are already intersecting.
    constexpr static auto m_threshold = contact_breaking_threshold * 4;
    constexpr static auto m_aabb_offset = vector3_one * -m_threshold;
    constexpr static auto m_separation_threshold = m_threshold * 1.3;

    entity_pair_vector intersect_islands(const tree_view &tree_viewA, const tree_view &tree_viewB) const;
    entity_pair_vector intersect_islands_a(const tree_view &tree_viewA, const tree_view &tree_viewB) const;
    entity_pair_vector intersect_island_np(const tree_view &island_tree, entt::entity np_entity) const;
    entity_pair_vector find_intersecting_islands(entt::entity island_entityA) const;

public:
    broadphase_main(entt::registry &);
    void update();

    void on_construct_tree_view(entt::registry &, entt::entity);
    void on_construct_static_tag(entt::registry &, entt::entity);
    void on_construct_static_kinematic_tag(entt::registry &, entt::entity);
    void on_destroy_node_id(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;
    dynamic_tree m_island_tree; // Tree for island AABBs.
    dynamic_tree m_np_tree; // Tree for non-procedural entities.
    contact_manifold_map m_manifold_map;
    std::vector<entity_pair_vector> m_pair_results;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_MAIN_HPP
