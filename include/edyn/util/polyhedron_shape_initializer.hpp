#ifndef EDYN_UTIL_POLYHEDRON_SHAPE_INITIALIZER_HPP
#define EDYN_UTIL_POLYHEDRON_SHAPE_INITIALIZER_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>

namespace edyn {

/**
 * @brief Sets up rotated meshes for polyhedrons that have been recently
 * created, including polyhedrons which reside in compound shapes.
 */
class polyhedron_shape_initializer {
public:
    polyhedron_shape_initializer(entt::registry &);
    ~polyhedron_shape_initializer();

    void on_construct_polyhedron_shape(entt::registry &, entt::entity);
    void on_construct_compound_shape(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    void init_new_shapes();

private:
    entt::registry *m_registry;
    std::vector<entt::entity> m_new_polyhedron_shapes;
    std::vector<entt::entity> m_new_compound_shapes;
    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_UTIL_POLYHEDRON_SHAPE_INITIALIZER_HPP
