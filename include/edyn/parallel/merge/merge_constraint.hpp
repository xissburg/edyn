#ifndef EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP

#include "edyn/constraints/constraint.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(constraint_base &new_comp, const entity_map &emap) {
    for (auto &entity : new_comp.body) {
        entity = emap.remloc(entity);
    }
}

template<> inline
void merge(point_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(distance_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(soft_distance_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(hinge_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(generic_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(contact_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(cvjoint_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(cone_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(null_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

template<> inline
void merge(gravity_constraint &new_comp, const entity_map &emap) {
    merge(static_cast<constraint_base &>(new_comp), emap);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
