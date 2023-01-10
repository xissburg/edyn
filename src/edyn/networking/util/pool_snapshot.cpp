#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/serialization/entt_s11n.hpp"
#include "edyn/serialization/math_s11n.hpp"
#include "edyn/serialization/std_s11n.hpp"

namespace edyn {

std::unique_ptr<pool_snapshot_data>(*g_make_pool_snapshot_data)(component_index_type) =
    create_make_pool_snapshot_data_function(networked_components);

}
