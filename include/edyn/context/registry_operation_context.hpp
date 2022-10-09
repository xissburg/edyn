#ifndef EDYN_CONTEXT_REGISTRY_OPERATION_CONTEXT_HPP
#define EDYN_CONTEXT_REGISTRY_OPERATION_CONTEXT_HPP

#include "edyn/replication/make_reg_op_builder.hpp"

namespace edyn {

std::unique_ptr<registry_operation_builder> make_reg_op_builder_default(entt::registry &);
std::unique_ptr<registry_operation_observer> make_reg_op_observer_default(registry_operation_builder &);

struct registry_operation_context {
    make_reg_op_builder_func_t make_reg_op_builder {&make_reg_op_builder_default};
    make_reg_op_observer_func_t make_reg_op_observer {&make_reg_op_observer_default};
};

}

#endif // EDYN_CONTEXT_REGISTRY_OPERATION_CONTEXT_HPP
