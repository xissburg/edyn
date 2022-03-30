#ifndef EDYN_COLLISION_BROADPHASE_MAIN_HPP
#define EDYN_COLLISION_BROADPHASE_MAIN_HPP

#include <map>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/utility.hpp>
#include "edyn/comp/island.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/util/entity_pair.hpp"

namespace edyn {

class tree_view;

class broadphase_main {

    using aabb_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, AABB>;
    using multi_resident_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, multi_island_resident>;
    using tree_view_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, tree_view>;

    // A higher threshold is used in the main broadphase to create contact
    // manifolds between different islands a little earlier and decrease the
    // probability they'll arrive in the corresponding island worker when the
    // shapes are already intersecting.
    constexpr static auto m_threshold = contact_breaking_threshold * scalar(4);
    constexpr static auto m_aabb_offset = vector3_one * -m_threshold;
    constexpr static auto m_separation_threshold = m_threshold * scalar(1.3);

    entity_pair_vector intersect_islands(const tree_view &tree_viewA, const tree_view &tree_viewB,
                                         const aabb_view_t &aabb_view) const;
    entity_pair_vector intersect_islands_a(const tree_view &tree_viewA, const tree_view &tree_viewB,
                                           const aabb_view_t &aabb_view) const;
    entity_pair_vector intersect_island_np(const tree_view &island_tree, entt::entity np_entity,
                                           const aabb_view_t &aabb_view) const;
    entity_pair_vector find_intersecting_islands(entt::entity island_entityA,
                                                 const aabb_view_t &aabb_view,
                                                 const multi_resident_view_t &resident_view,
                                                 const tree_view_view_t &tree_view_view) const;

public:
    broadphase_main(entt::registry &);
    void update();

    template<typename Func>
    void query_islands(const AABB &aabb, Func func);

    template<typename Func>
    void query_non_procedural(const AABB &aabb, Func func);

    template<typename Func>
    void raycast_islands(vector3 p0, vector3 p1, Func func);

    template<typename Func>
    void raycast_non_procedural(vector3 p0, vector3 p1, Func func);

    void on_construct_tree_view(entt::registry &, entt::entity);
    void on_construct_static_kinematic_tag(entt::registry &, entt::entity);
    void on_construct_aabb(entt::registry &, entt::entity);
    void on_destroy_tree_resident(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;
    dynamic_tree m_island_tree; // Tree for island AABBs.
    dynamic_tree m_np_tree; // Tree for non-procedural entities.
    std::vector<entity_pair_vector> m_pair_results;

    bool should_collide(entt::entity, entt::entity) const;
};

template<typename Func>
void broadphase_main::query_islands(const AABB &aabb, Func func) {
    m_island_tree.query(aabb, [&] (tree_node_id_t id) {
        func(m_island_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase_main::query_non_procedural(const AABB &aabb, Func func) {
    m_np_tree.query(aabb, [&] (tree_node_id_t id) {
        func(m_np_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase_main::raycast_islands(vector3 p0, vector3 p1, Func func) {
    m_island_tree.raycast(p0, p1, [&] (tree_node_id_t id) {
        func(m_island_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase_main::raycast_non_procedural(vector3 p0, vector3 p1, Func func) {
    m_np_tree.raycast(p0, p1, [&] (tree_node_id_t id) {
        func(m_np_tree.get_node(id).entity);
    });
}

}

#endif // EDYN_COLLISION_BROADPHASE_MAIN_HPP
