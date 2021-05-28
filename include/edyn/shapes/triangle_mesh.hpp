#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <cstdint>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/util/triangle_util.hpp"
#include "edyn/collision/static_tree.hpp"

namespace edyn {

template<typename T>
struct commutative_pair {
    T first;
    T second;

    commutative_pair() = default;
    commutative_pair(T first, T second)
        : first(first)
        , second(second)
    {}

    auto operator[](size_t idx) const {
        EDYN_ASSERT(idx < 2);
        return idx == 0 ? first : second;
    }

    bool operator==(const commutative_pair &other) const {
        return (first == other.first && second == other.second) ||
               (first == other.second && second == other.first);
    }

    bool operator<(const commutative_pair &other) const {
        if (first == other.second && second == other.first) {
            return false;
        }
        if (first == other.first) {
            return second < other.second;
        }
        return first < other.first;
    }
};

template<typename T>
class flat_nested_array {
public:
    class inner_array {
    public:
        inner_array(const flat_nested_array<T> *parent, size_t range_start, size_t size)
            : m_parent(parent)
            , m_range_start(range_start)
            , m_size(size)
        {}

        const T & operator[](size_t idx) const {
            EDYN_ASSERT(idx < m_size);
            return m_parent->m_data[m_range_start + idx];
        }

        size_t size() const {
            return m_size;
        }

    private:
        const flat_nested_array<T> *m_parent;
        size_t m_range_start;
        size_t m_size;
    };

    void push_array() {
        m_range_starts.push_back(m_data.size());
    }

    void push_back(const T &value) {
        m_data.push_back(value);
    }

    size_t size() const {
        return m_range_starts.size();
    }

    void reserve(size_t count) {
        m_data.reserve(count);
    }

    inner_array operator[](size_t idx) const {
        EDYN_ASSERT(idx < m_range_starts.size());
        auto next_idx = idx + 1;
        auto range_start = m_range_starts[idx];
        auto range_end = next_idx < m_range_starts.size() ? m_range_starts[next_idx] : m_data.size();
        auto range_size = range_end - range_start;
        return inner_array(this, range_start, range_size);
    }

    template<typename Archive, typename U>
    friend void serialize(Archive &, flat_nested_array<U> &);

    template<typename U>
    friend size_t serialization_sizeof(const flat_nested_array<U> &);

private:
    std::vector<T> m_data;
    std::vector<size_t> m_range_starts;
};

/**
 * @brief A triangle mesh. Includes adjacency information and a tree to
 * accelerate closest point queries.
 */
class triangle_mesh {
public:
    void initialize();
    void calculate_face_normals();
    void init_edge_indices();
    void calculate_edge_normals();
    void init_vertex_tangents();
    void calculate_convex_edges();
    void build_triangle_tree();

public:
    using index_type = uint32_t;

    size_t num_vertices() const {
        return m_vertices.size();
    }

    size_t num_edges() const {
        return m_edge_vertex_indices.size();
    }

    size_t num_triangles() const {
        return m_indices.size();
    }

    AABB get_aabb() const {
        return m_triangle_tree.root_aabb();
    }

    vector3 get_vertex_position(size_t vertex_idx) const {
        EDYN_ASSERT(vertex_idx < m_vertices.size());
        return m_vertices[vertex_idx];
    }

    triangle_vertices get_triangle_vertices(size_t tri_idx) const;

    vector3 get_triangle_normal(size_t tri_idx) const {
        EDYN_ASSERT(tri_idx < m_normals.size());
        return m_normals[tri_idx];
    }

    std::array<vector3, 2> get_edge_vertices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_vertex_indices.size());
        return {
            m_vertices[m_edge_vertex_indices[edge_idx][0]],
            m_vertices[m_edge_vertex_indices[edge_idx][1]]
        };
    }

    auto get_edge_face_indices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_face_indices.size());
        return m_edge_face_indices[edge_idx];
    }

    auto get_edge_normals(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_normals.size());
        return m_edge_normals[edge_idx];
    }

    /**
     * @brief Returns the normals of the faces that share an edge
     * which is assumed to be convex.
     * @param edge_idx Edge index.
     * @return Two face normals.
     */
    std::array<vector3, 2> get_convex_edge_face_normals(size_t edge_idx) const ;

    template<typename Func>
    void visit_triangles(const AABB &aabb, Func func) const {
        m_triangle_tree.visit(aabb, func);
    }

    template<typename Func>
    void visit_all(Func func) const {
        for (size_t i = 0; i < num_triangles(); ++i) {
            auto verts = triangle_vertices{
                m_vertices[m_indices[i][0]],
                m_vertices[m_indices[i][1]],
                m_vertices[m_indices[i][2]]
            };

            func(i, verts);
        }
    }

    bool in_vertex_voronoi(size_t vertex_idx, const vector3 &dir) const;

    bool in_edge_voronoi(size_t edge_idx, const vector3 &dir) const;

    bool is_convex_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_convex_edge.size());
        return m_is_convex_edge[edge_idx];
    }

    bool is_boundary_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_boundary_edge.size());
        return m_is_boundary_edge[edge_idx];
    }

    bool ignore_triangle_feature(size_t tri_idx, triangle_feature tri_feature,
                                 size_t feature_idx, const vector3 &dir) const;

    index_type get_face_vertex_index(size_t tri_idx, size_t vertex_idx) const {
        EDYN_ASSERT(tri_idx < m_face_edge_indices.size());
        EDYN_ASSERT(vertex_idx < 3);
        return m_indices[tri_idx][vertex_idx];
    }

    index_type get_face_edge_index(size_t tri_idx, size_t edge_idx) const {
        EDYN_ASSERT(tri_idx < m_face_edge_indices.size());
        EDYN_ASSERT(edge_idx < 3);
        return m_face_edge_indices[tri_idx][edge_idx];
    }

//private:
    // Vertex positions.
    std::vector<vector3> m_vertices;

    // Vertex indices for each triangular face. Each element represents the
    // vertex indices of one triangle.
    std::vector<std::array<index_type, 3>> m_indices;

    // Face normals.
    std::vector<vector3> m_normals;

    // Vertex indices for each unique edge. Each pair of values represent the
    // vertex indices for one edge.
    std::vector<commutative_pair<index_type>> m_edge_vertex_indices;

    flat_nested_array<index_type> m_vertex_edge_indices;

    // Vectors that are orthogonal to an edge and parallel to the faces that
    // share this edge. They're stored in pairs where each value is a vector
    // that points towards each face. For perimetral edges, both values are the
    // same and point towards the single face that contains this edge.
    std::vector<std::array<vector3, 2>> m_edge_normals;

    // Vectors that are tangent to a vertex. More specifically, these are the
    // directions of the edges that depart from a vertex.
    flat_nested_array<vector3> m_vertex_tangents;

    // Each element represents the indices of the three edges of a face.
    std::vector<std::array<index_type, 3>> m_face_edge_indices;

    // Indices of the two faces that share the i-th edge. Perimetral edges will
    // have the same value for both faces.
    std::vector<std::array<index_type, 2>> m_edge_face_indices;

    // Indicates whether an edge is at the boundary. These edges are associated
    // with a single triangle.
    std::vector<bool> m_is_boundary_edge;

    // Whether an edge is convex.
    std::vector<bool> m_is_convex_edge;

    static_tree m_triangle_tree;
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP
