#ifndef EDYN_UTIL_CONSTRAINT_UTIL_HPP
#define EDYN_UTIL_CONSTRAINT_UTIL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct contact_manifold;
struct constraint_row;
struct constraint_row_options;
struct matrix3x3;

namespace internal {
    bool pre_make_constraint(entt::entity entity, entt::registry &registry,
                             entt::entity body0, entt::entity body1);
}

/**
 * @brief Assigns a constraint component of type `T` to the given entity and does
 * all the other necessary steps to tie things together correctly.
 *
 * @tparam T Constraint type.
 * @param entity The constraint entity.
 * @param registry The `entt::registry`.
 * @param body0 First rigid body entity.
 * @param body1 Second rigid body entity.
 * graph.
 */
template<typename T>
T & make_constraint(entt::entity entity, entt::registry &registry,
                    entt::entity body0, entt::entity body1) {

    auto is_new = internal::pre_make_constraint(entity, registry, body0, body1);
    auto &con = registry.emplace<T>(entity, body0, body1);
    auto &con_dirty = registry.get_or_emplace<dirty>(entity);
    con_dirty.created<T>();

    if (is_new) {
        con_dirty.set_new();
    }

    return con;
}

/*! @copydoc make_constraint */
template<typename T>
auto make_constraint(entt::registry &registry,
                     entt::entity body0, entt::entity body1) {
    auto ent = registry.create();
    auto &con = make_constraint<T>(ent, registry, body0, body1);
    return std::pair<entt::entity, T &>(ent, con);
}

entt::entity make_contact_manifold(entt::registry &,
                                   entt::entity body0, entt::entity body1,
                                   scalar separation_threshold);

void make_contact_manifold(entt::entity contact_entity, entt::registry &,
                           entt::entity body0, entt::entity body1,
                           scalar separation_threshold);

void swap_manifold(contact_manifold &manifold);

scalar get_effective_mass(const constraint_row &);

scalar get_effective_mass(const std::array<vector3, 4> &J,
                          scalar inv_mA, const matrix3x3 &inv_IA,
                          scalar inv_mB, const matrix3x3 &inv_IB);

scalar get_relative_speed(const std::array<vector3, 4> &J,
                          const vector3 &linvelA,
                          const vector3 &angvelA,
                          const vector3 &linvelB,
                          const vector3 &angvelB);

void prepare_row(constraint_row &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA,
                 const vector3 &linvelB, const vector3 &angvelB);

void apply_impulse(scalar impulse, constraint_row &row);

void warm_start(constraint_row &row);

}

#endif // EDYN_UTIL_CONSTRAINT_UTIL_HPP
