#ifndef EDYN_UTIL_MAKE_REG_OP_BUILDER_HPP
#define EDYN_UTIL_MAKE_REG_OP_BUILDER_HPP

#include <memory>
#include <entt/entity/fwd.hpp>

namespace edyn {

class registry_operation_builder;

/**
 * @brief Function type of a factory function that creates instances of a
 * registry operations builder implementation.
 */
using make_reg_op_builder_func_t = std::unique_ptr<registry_operation_builder>(*)();

/**
 * @brief Creates a new registry operation builder.
 *
 * Returns a builder implementation that supports handling all shared component
 * types plus any external component set by the user.
 *
 * @return Safe pointer to an instance of a builder implementation.
 */
std::unique_ptr<registry_operation_builder> make_reg_op_builder(entt::registry &registry);

}

#endif // EDYN_UTIL_MAKE_REG_OP_BUILDER_HPP
