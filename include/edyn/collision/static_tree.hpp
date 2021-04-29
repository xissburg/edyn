#ifndef EDYN_COLLISION_STATIC_TREE_HPP
#define EDYN_COLLISION_STATIC_TREE_HPP

#include "edyn/comp/aabb.hpp"
#include <vector>
#include <iterator>
#include <numeric>
#include <algorithm>

namespace edyn {

constexpr uint32_t EDYN_NULL_NODE = UINT32_MAX;

namespace detail {
    template<typename Iterator_AABB, typename Iterator_ids>
    Iterator_ids aabb_set_partition(Iterator_AABB aabb_begin, Iterator_AABB aabb_end, 
                                    Iterator_ids ids_begin, Iterator_ids ids_end,
                                    const AABB &set_aabb) {
        auto aabb_size = set_aabb.max - set_aabb.min;
        auto split_axis_idx = max_index(aabb_size);
        auto split_pos = set_aabb.center()[split_axis_idx];

        std::sort(ids_begin, ids_end, [&] (auto a, auto b) {
            auto &b0 = *(aabb_begin + a);
            auto &b1 = *(aabb_begin + b);
            auto ca = b0.center();
            auto cb = b1.center();
            return ca[split_axis_idx] < cb[split_axis_idx];
        });

        for (auto it = ids_begin; it != ids_end; ++it) {
            auto &aabb = *(aabb_begin + *it);
            auto center = aabb.center();

            if (center[split_axis_idx] > split_pos) {
                return it;
            }
        }

        return ids_begin + std::distance(ids_begin, ids_end) / 2;
    }
}

class static_tree {
public:
    struct tree_node {
        AABB aabb;
        uint32_t child1;
        union {
            uint32_t child2;
            uint32_t id;
        };

        bool is_leaf() const {
            return child1 == EDYN_NULL_NODE;
        }
    };

    AABB root_aabb() const {
        EDYN_ASSERT(!m_nodes.empty());
        return m_nodes.front().aabb;
    }

    bool empty() const {
        return m_nodes.empty();
    }

    template<typename Func>
    void visit(const AABB &aabb, Func func) const {
        std::vector<uint32_t> stack;
        uint32_t root_node_idx = 0;
        stack.push_back(root_node_idx);

        while (!stack.empty()) {
            auto node_idx = stack.back();

            if (node_idx == EDYN_NULL_NODE) {
                continue;
            }

            stack.pop_back();

            auto &node = m_nodes[node_idx];

            if (intersect(node.aabb, aabb)) {
                if (node.is_leaf()) {
                    func(node.id);
                } else {
                    stack.push_back(node.child1);
                    stack.push_back(node.child2);
                }
            }
        }
    }

    template<typename Iterator, typename Func>
    void build(Iterator aabb_begin, Iterator aabb_end, Func &report_leaf, uint32_t max_obj_per_leaf = 1) {
        EDYN_ASSERT(aabb_begin != aabb_end);

        auto count = std::distance(aabb_begin, aabb_end);
        std::vector<uint32_t> ids(count);
        std::iota(ids.begin(), ids.end(), 0);

        // Insert root node.
        m_nodes.emplace_back();

        recurse_build(aabb_begin, aabb_end, ids.begin(), ids.end(), 
                      0, report_leaf, max_obj_per_leaf);
    }

    template<typename Iterator_AABB, typename Iterator_ids, typename Func>
    void recurse_build(Iterator_AABB aabb_begin, Iterator_AABB aabb_end,
                       Iterator_ids ids_begin, Iterator_ids ids_end,
                       size_t node_idx, Func &report_leaf,
                       uint32_t max_obj_per_leaf) {
        EDYN_ASSERT(aabb_begin != aabb_end);

        AABB set_aabb = *(aabb_begin + *ids_begin);

        for (auto it = ids_begin + 1; it != ids_end; ++it) {
            set_aabb = enclosing_aabb(set_aabb, *(aabb_begin + *it));
        }

        auto &node = m_nodes[node_idx];
        node.aabb = set_aabb;

        auto count = std::distance(ids_begin, ids_end);

        if (count <= max_obj_per_leaf) {
            node.child1 = EDYN_NULL_NODE;
            report_leaf(node, ids_begin, ids_end);
        } else {
            auto ids_middle = detail::aabb_set_partition(aabb_begin, aabb_end, ids_begin, ids_end, set_aabb);

            auto child1 = m_nodes.size();
            auto child2 = m_nodes.size() + 1;

            node.child1 = child1;
            node.child2 = child2;

            m_nodes.emplace_back();
            m_nodes.emplace_back();

            recurse_build(aabb_begin, aabb_end, ids_begin, ids_middle, 
                          child1, report_leaf, max_obj_per_leaf);
            recurse_build(aabb_begin, aabb_end, ids_middle, ids_end, 
                          child2, report_leaf, max_obj_per_leaf);
        }
    }

    std::vector<tree_node> m_nodes;
};

}

#endif // EDYN_COLLISION_STATIC_TREE_HPP