#ifndef EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

/**
 * @brief A constraint that does nothing. It is primarily used to connect
 * related or dependent entities together in the entity graph so they'll
 * always be present together in the same island.
 */
struct null_constraint : public constraint_base {};

template<typename Archive>
void serialize(Archive &archive, null_constraint &con) {
    archive(con.body);
}

}

#endif // EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP
