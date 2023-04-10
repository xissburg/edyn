#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/serialization/entt_s11n.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/math_s11n.hpp"

namespace edyn {

static std::unique_ptr<extrapolation_modified_comp>
make_extrapolation_modified_comp_default(entt::registry &registry) {
    return std::unique_ptr<extrapolation_modified_comp>(
        new extrapolation_modified_comp_impl(registry, networked_components));
}

client_network_context::client_network_context(entt::registry &registry)
    : snapshot_importer(new client_snapshot_importer_impl(networked_components))
    , snapshot_exporter(new client_snapshot_exporter_impl(registry, networked_components, {}))
    , input_history{}
    , make_extrapolation_modified_comp(&make_extrapolation_modified_comp_default)
{
    clock_sync.send_packet.connect<&entt::sigh<packet_observer_func_t>::publish>(packet_signal);
}

}
