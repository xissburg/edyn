#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/extrapolation_job.hpp"

namespace edyn {

server_network_context::server_network_context()
    : snapshot_importer(new server_snapshot_importer_impl(networked_components, {}))
    , snapshot_exporter(new server_snapshot_exporter_impl(networked_components, transient_components, {}))
{}

}
