#include "edyn/context/settings.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder_default() {
    return std::unique_ptr<registry_operation_builder>(
        new registry_operation_builder_impl(shared_components));
}

}
