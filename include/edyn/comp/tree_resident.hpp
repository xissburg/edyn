#ifndef EDYN_COMP_TREE_RESIDENT_HPP
#define EDYN_COMP_TREE_RESIDENT_HPP

#include "edyn/collision/tree_node.hpp"

namespace edyn {

struct tree_resident {
    tree_node_id_t id;
    bool procedural;
};

}

#endif // EDYN_COMP_TREE_RESIDENT_HPP
