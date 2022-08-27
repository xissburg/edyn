#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/serialization/entt_s11n.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/math_s11n.hpp"

namespace edyn {

server_network_context::server_network_context(entt::registry &registry)
    : snapshot_importer(new server_snapshot_importer_impl(networked_components, {}))
    , snapshot_exporter(new server_snapshot_exporter_impl(registry, networked_components))
{}

}
