#ifndef EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP
#define EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP

#include "edyn/collision/contact_manifold.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

class entity_map;

void import_contact_manifolds(entt::registry &registry, const entity_map &emap,
                              const std::vector<extrapolation_result::contact_manifold_info> &manifolds,
                              const entt::storage<extrapolation_result::contact_point_info> &contacts);

void import_contact_manifolds(entt::registry &registry,
                              const std::vector<extrapolation_result::contact_manifold_info> &manifolds,
                              const entt::storage<extrapolation_result::contact_point_info> &contacts);

}

#endif // EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP
