#ifndef EDYN_COLLISION_BROADPHASE_WORKER_HPP
#define EDYN_COLLISION_BROADPHASE_WORKER_HPP

#include <vector>
#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/util/entity_pair.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/collision/contact_manifold_map.hpp"

namespace edyn {

struct job;

class broadphase_worker {
    // Offset applied to AABBs when querying the trees.
    constexpr static auto m_aabb_offset = vector3_one * -contact_breaking_threshold;

    // Separation threshold for new manifolds.
    constexpr static auto m_separation_threshold = contact_breaking_threshold * scalar{4 * 1.3};

    void init_new_aabb_entities();
    bool should_collide(entt::entity, entt::entity) const;

    void collide_tree(const dynamic_tree &tree, entt::entity entity, const AABB &offset_aabb);
    void collide_tree_async(const dynamic_tree &tree, entt::entity entity, const AABB &offset_aabb, size_t result_index);

    void common_update();

public:

    broadphase_worker(entt::registry &);
    bool parallelizable() const;
    void update();
    void update_async(job &completion_job);
    void finish_async_update();

    /**
     * @brief Returns a view of the procedural dynamic tree.
     * @return Tree view of the procedural dynamic tree.
     */
    tree_view view() const;

    void on_construct_aabb(entt::registry &, entt::entity);
    void on_destroy_node_id(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;
    dynamic_tree m_tree; // Procedural dynamic tree.
    dynamic_tree m_np_tree; // Non-procedural dynamic tree.
    contact_manifold_map m_manifold_map;
    std::vector<entt::entity> m_new_aabb_entities;
    std::vector<entity_pair_vector> m_pair_results;
};

}

#endif // EDYN_COLLISION_BROADPHASE_WORKER_HPP