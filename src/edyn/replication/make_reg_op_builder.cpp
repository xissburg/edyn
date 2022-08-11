#include "edyn/replication/make_reg_op_builder.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/config/config.h"
#include <entt/entity/registry.hpp>

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    EDYN_ASSERT(settings.make_reg_op_builder != nullptr);
    return (*settings.make_reg_op_builder)(registry);
}

std::unique_ptr<registry_operation_observer> make_reg_op_observer(registry_operation_builder &builder) {
    auto &registry = builder.get_registry();
    auto &settings = registry.ctx().at<edyn::settings>();
    EDYN_ASSERT(settings.make_reg_op_observer != nullptr);
    return (*settings.make_reg_op_observer)(builder);
}

}
