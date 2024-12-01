#ifndef EDYN_COLLISION_BROADPHASE_HPP
#define EDYN_COLLISION_BROADPHASE_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/core/entity_pair.hpp"
#include "edyn/collision/dynamic_tree.hpp"

namespace edyn {

class broadphase final {
    // Offset applied to AABBs when querying the trees.
    constexpr static auto m_aabb_offset = vector3_one * -contact_breaking_threshold;

    // Separation threshold for new manifolds.
    constexpr static auto m_separation_threshold = contact_breaking_threshold * scalar(1.3);

    void move_aabbs();
    void destroy_separated_manifolds();

    void collide_tree(const dynamic_tree &tree, entt::entity entity, const AABB &offset_aabb) const;
    void collide_tree_async(const dynamic_tree &tree, entt::entity entity, const AABB &offset_aabb, size_t result_index);
    void collide_parallel();
    void finish_collide();

    void on_construct_aabb(entt::registry &, entt::entity);
    void on_destroy_aabb(entt::registry &, entt::entity);
    void on_destroy_tree_resident(entt::registry &, entt::entity);
    void on_construct_island_aabb(entt::registry &, entt::entity);
    void on_destroy_island_tree_resident(entt::registry &, entt::entity);

    void collide_parallel_task(unsigned start, unsigned end);

public:
    broadphase(entt::registry &);
    broadphase(const broadphase &) = delete;
    broadphase & operator=(const broadphase &) = delete;

    void init_new_aabb_entities();
    void update(bool mt);

    template<typename Func>
    void raycast(vector3 p0, vector3 p1, Func func) const;

    template<typename Func>
    void query_procedural(const AABB &aabb, Func func) const;

    template<typename Func>
    void query_non_procedural(const AABB &aabb, Func func) const;

    template<typename Func>
    void query_islands(const AABB &aabb, Func func) const;

    void clear();

    void set_procedural(entt::entity, bool);

private:
    entt::registry *m_registry;
    dynamic_tree m_tree; // Procedural dynamic tree.
    dynamic_tree m_np_tree; // Non-procedural dynamic tree.
    dynamic_tree m_island_tree; // Island AABB tree.
    std::vector<entt::entity> m_new_aabb_entities;
    std::vector<entity_pair_vector> m_pair_results;
    size_t m_max_sequential_size {8};
    std::vector<entt::scoped_connection> m_connections;
};

template<typename Func>
void broadphase::raycast(vector3 p0, vector3 p1, Func func) const {
    m_tree.raycast(p0, p1, [&](tree_node_id_t id) {
        func(m_tree.get_node(id).entity);
    });
    m_np_tree.raycast(p0, p1, [&](tree_node_id_t id) {
        func(m_np_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase::query_procedural(const AABB &aabb, Func func) const {
    m_tree.query(aabb, [&](tree_node_id_t id) {
        func(m_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase::query_non_procedural(const AABB &aabb, Func func) const {
    m_np_tree.query(aabb, [&](tree_node_id_t id) {
        func(m_np_tree.get_node(id).entity);
    });
}

template<typename Func>
void broadphase::query_islands(const AABB &aabb, Func func) const {
    m_island_tree.query(aabb, [&](tree_node_id_t id) {
        func(m_island_tree.get_node(id).entity);
    });
}

}

#endif // EDYN_COLLISION_BROADPHASE_HPP
