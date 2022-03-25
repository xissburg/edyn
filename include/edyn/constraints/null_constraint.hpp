#ifndef EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

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

template<>
void prepare_constraints<null_constraint>(entt::registry &, row_cache &cache, scalar dt);

}

#endif // EDYN_CONSTRAINTS_NULL_CONSTRAINT_HPP
