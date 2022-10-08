#ifndef EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP
#define EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP

#include "edyn/collision/contact_manifold.hpp"
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

struct contact_manifold;
class entity_map;

void import_contact_manifolds(entt::registry &registry, const entity_map &emap,
                              const std::vector<contact_manifold> &manifolds);

void import_contact_manifolds(entt::registry &registry,
                              const std::vector<contact_manifold> &manifolds);

}

#endif // EDYN_NETWORKING_UTIL_IMPORT_CONTACT_MANIFOLDS_HPP
