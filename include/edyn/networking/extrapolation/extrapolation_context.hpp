#ifndef EDYN_NETWORKING_EXTRAPOLATION_CONTEXT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_CONTEXT_HPP

#include <entt/entity/registry.hpp>

namespace edyn {

/**
 * @brief Context variables which will be made available during extrapolation.
 */
class extrapolation_context : public entt::registry::context  {
public:
    extrapolation_context() : entt::registry::context(entt::registry::allocator_type{}) {}
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_CONTEXT_HPP
