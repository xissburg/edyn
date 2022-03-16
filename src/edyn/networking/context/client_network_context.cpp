#include "edyn/comp/shared_comp.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/extrapolation_job.hpp"

namespace edyn {

client_network_context::client_network_context()
    : pool_snapshot_importer(new client_pool_snapshot_importer_impl(networked_components))
    , pool_snapshot_exporter(new client_pool_snapshot_exporter_impl(networked_components, transient_components, {}))
    , is_input_component_func([] (entt::id_type) { return false; })
    , state_history(std::make_shared<comp_state_history>())
{
    clock_sync.send_packet.connect<&entt::sigh<packet_observer_func_t>::publish>(packet_signal);
}

}
