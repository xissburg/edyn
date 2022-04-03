#include "edyn/util/make_reg_op_builder.hpp"
#include "edyn/util/registry_operation_builder.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/config/config.h"
#include <entt/entity/registry.hpp>

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    EDYN_ASSERT(settings.make_reg_op_builder != nullptr);
    return (*settings.make_reg_op_builder)();
}

}
