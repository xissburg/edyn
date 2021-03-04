#ifndef EDYN_COMP_GRAPH_EDGE_HPP
#define EDYN_COMP_GRAPH_EDGE_HPP

#include "edyn/parallel/entity_graph.hpp"

namespace edyn {

struct graph_edge {
    entity_graph::index_type edge_index;
};

}

#endif // EDYN_COMP_GRAPH_EDGE_HPP
