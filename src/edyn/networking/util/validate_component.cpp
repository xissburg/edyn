#include "edyn/networking/util/validate_component.hpp"

namespace edyn {

template<typename... Floats>
bool validate_floats(Floats ... floats) {
    return (std::isfinite(floats) && ...);
}

bool validate_vector3(vector3 v) {
    return validate_floats(v.x, v.y, v.z);
}

bool validate_quat(quaternion q) {
    return
        validate_floats(q.x, q.y, q.z, q.w) &&
        std::abs(length_sqr(q) - scalar(1)) < 0.001;

}

bool validate_mat(const matrix3x3 &m) {
    return validate_vector3(m.row[0]) && validate_vector3(m.row[1]) && validate_vector3(m.row[2]);
}

bool validate_component(const entt::registry &registry, entt::entity entity, AABB &aabb) {
    return validate_vector3(aabb.min) && validate_vector3(aabb.max);
}

bool validate_component(const entt::registry &registry, entt::entity entity, collision_filter &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, collision_exclusion &excl) {
    for (auto i = excl.num_entities(); i > 0;) {
        auto entity = excl.entity[--i];

        if (!registry.valid(entity)) {
            return false;
        }
    }

    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, inertia &i) {
    return validate_mat(i);
}

bool validate_component(const entt::registry &registry, entt::entity entity, inertia_inv &i) {
    return validate_mat(i);
}

bool validate_component(const entt::registry &registry, entt::entity entity, inertia_world_inv &i) {
    return validate_mat(i);
}

bool validate_component(const entt::registry &registry, entt::entity entity, gravity &g) {
    return validate_vector3(g);
}

bool validate_component(const entt::registry &registry, entt::entity entity, angvel &v) {
    return validate_vector3(v);
}

bool validate_component(const entt::registry &registry, entt::entity entity, linvel &v) {
    return validate_vector3(v);
}

bool validate_component(const entt::registry &registry, entt::entity entity, mass &m) {
    return std::isfinite(m);
}

bool validate_component(const entt::registry &registry, entt::entity entity, mass_inv &m) {
    return std::isfinite(m);
}

bool validate_component(const entt::registry &registry, entt::entity entity, material &m) {
    return
        validate_floats(m.restitution, m.friction, m.spin_friction, m.roll_friction, m.stiffness, m.damping) &&
        m.restitution >= 0 && m.restitution <= 1 &&
        m.friction >= 0 &&
        m.spin_friction >= 0 &&
        m.roll_friction >= 0 &&
        m.stiffness >= 0 &&
        m.damping >= 0;
}

bool validate_component(const entt::registry &registry, entt::entity entity, position &pos) {
    return validate_vector3(pos);
}

bool validate_component(const entt::registry &registry, entt::entity entity, orientation &orn) {
    return validate_quat(orn);
}

bool validate_component(const entt::registry &registry, entt::entity entity, continuous &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, center_of_mass &com) {
    return validate_vector3(com);
}

bool validate_component(const entt::registry &registry, entt::entity entity, dynamic_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, kinematic_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, static_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, procedural_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, sleeping_disabled_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, disabled_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, continuous_contacts_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, external_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, shape_index &sh_idx) {
    return sh_idx.value < std::tuple_size_v<decltype(shapes_tuple)>;
}

bool validate_component(const entt::registry &registry, entt::entity entity, rigidbody_tag &) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, const constraint_base &con) {
    return registry.valid(con.body[0]) && registry.valid(con.body[1]);
}

bool validate_component(const entt::registry &registry, entt::entity entity, null_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, gravity_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, point_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, distance_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, soft_distance_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, hinge_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, generic_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, cone_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, cvjoint_constraint &con) {
    return validate_component(registry, entity, std::as_const(static_cast<constraint_base &>(con)));
}

bool validate_component(const entt::registry &registry, entt::entity entity, entity_owner &owner) {
    return registry.valid(owner.client_entity);
}

bool validate_component(const entt::registry &registry, entt::entity entity, sphere_shape &sh) {
    return validate_floats(sh.radius);
}

bool validate_component(const entt::registry &registry, entt::entity entity, cylinder_shape &sh) {
    return validate_floats(sh.radius, sh.half_length);
}

bool validate_component(const entt::registry &registry, entt::entity entity, capsule_shape &sh) {
    return validate_floats(sh.radius, sh.half_length);
}

bool validate_component(const entt::registry &registry, entt::entity entity, box_shape &sh) {
    return validate_vector3(sh.half_extents);
}

bool validate_component(const entt::registry &registry, entt::entity entity, polyhedron_shape &sh) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, compound_shape &sh) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, plane_shape &sh) {
    if (!validate_floats(sh.constant, sh.normal.x, sh.normal.y, sh.normal.z)) {
        return false;
    }

    if (std::abs(length_sqr(sh.normal) - scalar(1)) > scalar(0.001)) {
        return false;
    }

    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, mesh_shape &sh) {
    return true;
}

bool validate_component(const entt::registry &registry, entt::entity entity, paged_mesh_shape &sh) {
    return true;
}

}