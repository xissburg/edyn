#include "edyn/util/polyhedron_shape_initializer.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/entt_util.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>

namespace edyn {

polyhedron_shape_initializer::polyhedron_shape_initializer(entt::registry &registry)
    : m_registry(&registry)
{
    m_connections.push_back(registry.on_construct<polyhedron_shape>().connect<&polyhedron_shape_initializer::on_construct_polyhedron_shape>(*this));
    m_connections.push_back(registry.on_update<polyhedron_shape>().connect<&polyhedron_shape_initializer::on_construct_polyhedron_shape>(*this));
    m_connections.push_back(registry.on_construct<compound_shape>().connect<&polyhedron_shape_initializer::on_construct_compound_shape>(*this));
    m_connections.push_back(registry.on_update<compound_shape>().connect<&polyhedron_shape_initializer::on_construct_compound_shape>(*this));
    m_connections.push_back(registry.on_destroy<rotated_mesh_list>().connect<&polyhedron_shape_initializer::on_destroy_rotated_mesh_list>(*this));
}

polyhedron_shape_initializer::~polyhedron_shape_initializer() {
    m_connections.clear();

    // New entities are created for compound shapes that contain more than one
    // polyhedron shape. Destroy those here to avoid dangling entities.
    auto rotated_view = m_registry->view<rotated_mesh_list>();

    for (auto entity : m_registry->view<compound_shape>()) {
        if (rotated_view.contains(entity)) {
            auto [rotated] = rotated_view.get(entity);
            auto next = rotated.next;

            while (next != entt::null) {
                auto after_next = std::get<0>(rotated_view.get(next)).next;
                m_registry->destroy(next);
                next = after_next;
            }
        }
    }
}

void polyhedron_shape_initializer::on_construct_polyhedron_shape(entt::registry &registry, entt::entity entity) {
    m_new_polyhedron_shapes.push_back(entity);
}

void polyhedron_shape_initializer::on_construct_compound_shape(entt::registry &registry, entt::entity entity) {
    m_new_compound_shapes.push_back(entity);
}

void polyhedron_shape_initializer::on_destroy_rotated_mesh_list(entt::registry &registry, entt::entity entity) {
    auto &rotated = registry.get<rotated_mesh_list>(entity);
    if (rotated.next != entt::null) {
        // Cascade delete. Could lead to mega tall call stacks.
        registry.destroy(rotated.next);
    }
}

void polyhedron_shape_initializer::init_new_shapes() {
    entity_vector_erase_invalid(m_new_polyhedron_shapes, *m_registry);
    entity_vector_erase_invalid(m_new_compound_shapes, *m_registry);

    if (m_new_polyhedron_shapes.empty() && m_new_compound_shapes.empty()) return;

    auto orn_view = m_registry->view<orientation>();
    auto polyhedron_view = m_registry->view<polyhedron_shape>();
    auto compound_view = m_registry->view<compound_shape>();

    for (auto entity : m_new_polyhedron_shapes) {
        auto [polyhedron] = polyhedron_view.get(entity);
        auto [orn] = orn_view.get(entity);
        // A new `rotated_mesh` is assigned to it, replacing another reference
        // that could be already in there, thus preventing concurrent access.
        auto rotated = make_rotated_mesh(*polyhedron.mesh, orn);
        auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
        polyhedron.rotated = rotated_ptr.get();
        m_registry->emplace_or_replace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr));
    }

    for (auto entity : m_new_compound_shapes) {
        auto [compound] = compound_view.get(entity);
        auto [orn] = orn_view.get(entity);
        auto prev_rotated_entity = entt::entity{entt::null};

        for (auto &node : compound.nodes) {
            if (!std::holds_alternative<polyhedron_shape>(node.shape_var)) continue;

            // Assign a `rotated_mesh_list` to this entity for the first
            // polyhedron and link it with more rotated meshes for the
            // remaining polyhedrons.
            auto &polyhedron = std::get<polyhedron_shape>(node.shape_var);
            auto local_orn = orn * node.orientation;
            auto rotated = make_rotated_mesh(*polyhedron.mesh, local_orn);
            auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
            polyhedron.rotated = rotated_ptr.get();

            if (prev_rotated_entity == entt::null) {
                m_registry->emplace_or_replace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr), node.orientation);
                prev_rotated_entity = entity;
            } else {
                auto next = m_registry->create();
                m_registry->emplace<rotated_mesh_list>(next, polyhedron.mesh, std::move(rotated_ptr), node.orientation);

                auto &prev_rotated_list = m_registry->get<rotated_mesh_list>(prev_rotated_entity);
                prev_rotated_list.next = next;
                prev_rotated_entity = next;
            }
        }
    }

    m_new_polyhedron_shapes.clear();
    m_new_compound_shapes.clear();
}

}
