#ifndef EDYN_UTIL_CONSTRAINT_UTIL_HPP
#define EDYN_UTIL_CONSTRAINT_UTIL_HPP

#include <entt/entt.hpp>
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/dirty.hpp"

namespace edyn {

struct constraint_row_data;

namespace internal {
    void pre_make_constraint(entt::entity entity, entt::registry &registry, 
                             entt::entity body0, entt::entity body1, bool is_graph_edge);
}

/**
 * @brief Assigns a constraint component containing the constraint `con` of type
 * `T` to the given entity and does all the other necessary setup to tie things
 * together correctly.
 * 
 * Provide a parent entity if this constraint should not be set up as an island
 * node.
 * 
 * @tparam T Constraint type.
 * @param entity The constraint entity.
 * @param registry The `entt::registry`.
 * @param con The constraint object.
 * @param body0 First rigid body entity.
 * @param body1 Second rigid body entity.
 * @param parent_entity Optional parent entity.
 */
template<typename T> inline
void make_constraint(entt::entity entity, entt::registry &registry, T&& con, 
                     entt::entity body0, entt::entity body1, bool is_graph_edge = true) {

    internal::pre_make_constraint(entity, registry, body0, body1, is_graph_edge);
    registry.emplace<constraint>(entity, std::array<entt::entity, 2>{body0, body1}, std::forward<T>(con));
}

/*! @copydoc make_constraint */
template<typename T> inline
entt::entity make_constraint(entt::registry &registry, T&& con, 
                             entt::entity ent0, entt::entity ent1, 
                             bool is_graph_edge = true) {
    auto ent = registry.create();
    make_constraint<T>(ent, registry, std::forward<T>(con), ent0, ent1, is_graph_edge);
    return ent;
}

/**
 * Gets the constraint of type `T` in the `constraint` component assigned
 * to `entity`.
 */
template<typename T> inline
T & get_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    return std::get<T>(con.var);
}

/**
 * Attempts to get a constraint of type `T` in the `constraint` component
 * assigned to `entity`.
 */
template<typename T> inline
T * try_get_constraint(entt::entity entity, entt::registry &registry) {
    auto *con = registry.try_get<constraint>(entity);
    if (con && std::holds_alternative<T>(con->var)) {
        return &std::get<T>(con->var);
    }
    return nullptr;
}

/**
 * @brief Adds one more constraint row to the given constraint. 
 * 
 * A new entity is created for the row with a `constraint_row` component and it
 * also gets an island node assigned to it which is then associated with the node
 * in the given entity. The new entity is appended to the row array in the 
 * `constraint` and it is also returned.
 * 
 * @param entity The constraint entity.
 * @param constraint A reference to the constraint component.
 * @param registry The `entt::registry`.
 * @param priority Constraint row priority.
 * @return The row entity.
 */
entt::entity add_constraint_row(entt::entity, constraint &, entt::registry &, int priority = 0);

entt::entity make_contact_manifold(entt::registry &, 
                                   entt::entity body0, entt::entity body1,
                                   scalar separation_threshold);

void make_contact_manifold(entt::entity contact_entity, entt::registry &, 
                           entt::entity body0, entt::entity body1, 
                           scalar separation_threshold);

/**
 * @brief Enables/disables a constraint and its rows.
 * 
 * @param entity The constraint entity.
 * @param registry The `entt::registry`.
 * @param enabled Whether the constraint should be enabled or disabled.
 */
void set_constraint_enabled(entt::entity, entt::registry &, bool enabled);

scalar get_effective_mass(const constraint_row_data &);

}

#endif // EDYN_UTIL_CONSTRAINT_UTIL_HPP