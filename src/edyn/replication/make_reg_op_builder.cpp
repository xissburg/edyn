#include "edyn/replication/make_reg_op_builder.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/config/config.h"
#include <entt/entity/registry.hpp>

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder(entt::registry &registry) {
    auto &ctx = registry.ctx().get<registry_operation_context>();
    EDYN_ASSERT(ctx.make_reg_op_builder != nullptr);
    return (*ctx.make_reg_op_builder)(registry);
}

std::unique_ptr<registry_operation_observer> make_reg_op_observer(registry_operation_builder &builder) {
    auto &registry = builder.get_registry();
    auto &ctx = registry.ctx().get<registry_operation_context>();
    EDYN_ASSERT(ctx.make_reg_op_observer != nullptr);
    return (*ctx.make_reg_op_observer)(builder);
}

}
