#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn {

std::unique_ptr<pool_snapshot_data>(*g_make_pool_snapshot_data)(unsigned) = create_make_pool_snapshot_data_function(networked_components);

}