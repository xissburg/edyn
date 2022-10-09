#include "edyn/context/registry_operation_context.hpp"
#include "edyn/comp/shared_comp.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder_default(entt::registry &registry) {
    return std::unique_ptr<registry_operation_builder>(
        new registry_operation_builder_impl(registry, shared_components_t{}));
}

std::unique_ptr<registry_operation_observer> make_reg_op_observer_default(registry_operation_builder &builder) {
    return std::unique_ptr<registry_operation_observer>(
        new registry_operation_observer_impl(builder, shared_components_t{}));
}

}
