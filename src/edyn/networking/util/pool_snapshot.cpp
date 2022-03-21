#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/serialization/s11n.hpp"

namespace edyn {

std::unique_ptr<pool_snapshot_data>(*g_make_pool_snapshot_data)(unsigned) = create_make_pool_snapshot_data_function(networked_components);

}
