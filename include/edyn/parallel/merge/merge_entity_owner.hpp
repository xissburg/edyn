#ifndef EDYN_PARALLEL_MERGE_MERGE_ENTITY_OWNER_HPP
#define EDYN_PARALLEL_MERGE_MERGE_ENTITY_OWNER_HPP

#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(entity_owner &new_comp, const entity_map &emap) {
    new_comp.client_entity = emap.remloc(new_comp.client_entity);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_ENTITY_OWNER_HPP
