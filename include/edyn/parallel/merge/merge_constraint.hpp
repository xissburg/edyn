#ifndef EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP

#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/merge/merge_component.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const constraint_base *old_comp, constraint_base &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }
}

template<merge_type MergeType>
void merge(const point_constraint *old_comp, point_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const distance_constraint *old_comp, distance_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const soft_distance_constraint *old_comp, soft_distance_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const hinge_constraint *old_comp, hinge_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const generic_constraint *old_comp, generic_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const contact_constraint *old_comp, contact_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const null_constraint *old_comp, null_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<merge_type MergeType>
void merge(const gravity_constraint *old_comp, gravity_constraint &new_comp, merge_context &ctx) {
    merge<MergeType>(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
