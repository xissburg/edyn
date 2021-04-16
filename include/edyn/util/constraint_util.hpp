#ifndef EDYN_UTIL_CONSTRAINT_UTIL_HPP
#define EDYN_UTIL_CONSTRAINT_UTIL_HPP

#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row;
struct constraint_row_options;

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
T & make_constraint(entt::entity entity, entt::registry &registry, 
                    entt::entity body0, entt::entity body1, bool is_graph_edge = true) {

    internal::pre_make_constraint(entity, registry, body0, body1, is_graph_edge);
    auto &con = registry.emplace<T>(entity, std::array<entt::entity, 2>{body0, body1});
    auto &con_dirty = registry.get_or_emplace<dirty>(entity);
    con_dirty.set_new().created<T>();
    return con;
}

/*! @copydoc make_constraint */
template<typename T> inline
std::pair<entt::entity, T&> make_constraint(entt::registry &registry, 
                             entt::entity ent0, entt::entity ent1, 
                             bool is_graph_edge = true) {
    auto ent = registry.create();
    auto &con = make_constraint<T>(ent, registry, ent0, ent1, is_graph_edge);
    return {ent, con};
}

entt::entity make_contact_manifold(entt::registry &, 
                                   entt::entity body0, entt::entity body1,
                                   scalar separation_threshold);

void make_contact_manifold(entt::entity contact_entity, entt::registry &, 
                           entt::entity body0, entt::entity body1, 
                           scalar separation_threshold);

scalar get_effective_mass(const constraint_row &);

void prepare_row(constraint_row &row, 
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &linvelB,
                 const vector3 &angvelA, const vector3 &angvelB);

void apply_impulse(scalar impulse, constraint_row &row);

void warm_start(constraint_row &row);

}

#endif // EDYN_UTIL_CONSTRAINT_UTIL_HPP