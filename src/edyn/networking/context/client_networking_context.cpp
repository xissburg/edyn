#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/util/client_pool_snapshot_importer.hpp"
#include "edyn/networking/extrapolation_job.hpp"

namespace edyn {

client_networking_context::client_networking_context()
    : pool_snapshot_importer(new client_pool_snapshot_importer_impl(networked_components))
{}

}