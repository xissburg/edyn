#include "edyn/networking/context/server_networking_context.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/extrapolation_job.hpp"

namespace edyn {

server_networking_context::server_networking_context()
    : pool_snapshot_importer(new server_pool_snapshot_importer_impl(networked_components, {}))
    , pool_snapshot_exporter(new server_pool_snapshot_exporter_impl(networked_components, transient_components))
{}

}