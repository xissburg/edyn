#include "edyn/comp/shared_comp.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/extrapolation_job.hpp"

namespace edyn {

client_network_context::client_network_context()
    : pool_snapshot_importer(new client_pool_snapshot_importer_impl(networked_components))
    , pool_snapshot_exporter(new client_pool_snapshot_exporter_impl(networked_components, transient_components, {}))
    , extrapolation_component_pool_import_func(internal::make_extrapolation_component_pools_import_func(shared_components))
    , extrapolation_component_pool_import_by_id_func(internal::make_extrapolation_component_pools_import_by_id_func(shared_components))
    , is_input_component_func([] (entt::id_type) { return false; })
    , state_history(std::make_shared<comp_state_history>())
{}

}
