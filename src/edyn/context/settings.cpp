#include "edyn/context/settings.hpp"
#include "edyn/comp/shared_comp.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/component_index_source.hpp"

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder_default(entt::registry &registry) {
    return std::unique_ptr<registry_operation_builder>(
        new registry_operation_builder_impl(registry, shared_components_t{}));
}

settings::settings()
    : index_source(new component_index_source_impl(shared_components_t{}))
{}

}
