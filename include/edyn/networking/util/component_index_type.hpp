#ifndef EDYN_NETWORKING_UTIL_COMPONENT_INDEX_TYPE_HPP
#define EDYN_NETWORKING_UTIL_COMPONENT_INDEX_TYPE_HPP

#include <cstdint>

namespace edyn {

// Index type of components stored in registry snapshot packets. Allow for
// up to 2^16 component types.
using component_index_type = uint16_t;

}

#endif // EDYN_NETWORKING_UTIL_COMPONENT_INDEX_TYPE_HPP
