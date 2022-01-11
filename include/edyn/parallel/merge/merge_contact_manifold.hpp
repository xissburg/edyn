#ifndef EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP

#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(contact_manifold &new_comp, entity_map &emap) {
    for (auto &entity : new_comp.body) {
        entity = emap.remloc(entity);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP
