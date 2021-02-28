#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include "edyn/util/entity_set.hpp"

namespace edyn {

/**
 * @brief An _island_ is a set of entities that can affect one another,
 * usually through constraints.
 */
struct island {

};

struct island_timestamp {
    double value;
};

struct island_container_single {
    entt::entity island_entity;
};

struct island_container {
    entity_set entities;
};

}

#endif // EDYN_COMP_ISLAND_HPP