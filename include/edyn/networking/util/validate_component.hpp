#ifndef EDYN_NETWORKING_VALIDATE_COMPONENT_HPP
#define EDYN_NETWORKING_VALIDATE_COMPONENT_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/comp/networked_comp.hpp"

namespace edyn {

bool validate_component(const entt::registry &registry, entt::entity entity, AABB &aabb);
bool validate_component(const entt::registry &registry, entt::entity entity, collision_filter &);
bool validate_component(const entt::registry &registry, entt::entity entity, collision_exclusion &excl);
bool validate_component(const entt::registry &registry, entt::entity entity, inertia &i);
bool validate_component(const entt::registry &registry, entt::entity entity, inertia_inv &i);
bool validate_component(const entt::registry &registry, entt::entity entity, inertia_world_inv &i);
bool validate_component(const entt::registry &registry, entt::entity entity, gravity &g);
bool validate_component(const entt::registry &registry, entt::entity entity, angvel &v);
bool validate_component(const entt::registry &registry, entt::entity entity, linvel &v);
bool validate_component(const entt::registry &registry, entt::entity entity, mass &m);
bool validate_component(const entt::registry &registry, entt::entity entity, mass_inv &m);
bool validate_component(const entt::registry &registry, entt::entity entity, material &m);
bool validate_component(const entt::registry &registry, entt::entity entity, position &pos);
bool validate_component(const entt::registry &registry, entt::entity entity, orientation &orn);
bool validate_component(const entt::registry &registry, entt::entity entity, continuous &);
bool validate_component(const entt::registry &registry, entt::entity entity, center_of_mass &com);
bool validate_component(const entt::registry &registry, entt::entity entity, dynamic_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, kinematic_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, static_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, procedural_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, sleeping_disabled_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, disabled_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, continuous_contacts_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, external_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, shape_index &sh_idx);
bool validate_component(const entt::registry &registry, entt::entity entity, rigidbody_tag &);
bool validate_component(const entt::registry &registry, entt::entity entity, null_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, gravity_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, point_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, distance_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, soft_distance_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, hinge_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, generic_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, cone_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, cvjoint_constraint &con);
bool validate_component(const entt::registry &registry, entt::entity entity, entity_owner &owner);
bool validate_component(const entt::registry &registry, entt::entity entity, sphere_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, cylinder_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, capsule_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, box_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, polyhedron_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, compound_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, plane_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, mesh_shape &sh);
bool validate_component(const entt::registry &registry, entt::entity entity, paged_mesh_shape &sh);

}

#endif // EDYN_NETWORKING_VALIDATE_COMPONENT_HPP
