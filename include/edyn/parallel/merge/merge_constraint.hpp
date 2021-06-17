#ifndef EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP

#include "edyn/constraints/constraint.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(const constraint_base *old_comp, constraint_base &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }
}

template<> inline
void merge(const point_constraint *old_comp, point_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const distance_constraint *old_comp, distance_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const soft_distance_constraint *old_comp, soft_distance_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const hinge_constraint *old_comp, hinge_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const generic_constraint *old_comp, generic_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const contact_constraint *old_comp, contact_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const null_constraint *old_comp, null_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const gravity_constraint *old_comp, gravity_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const contact_patch_constraint *old_comp, contact_patch_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const antiroll_constraint *old_comp, antiroll_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
    new_comp.third_entity = ctx.map->remloc(new_comp.third_entity);
}

template<> inline
void merge(const doublewishbone_constraint *old_comp, doublewishbone_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const tierod_constraint *old_comp, tierod_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const tirecarcass_constraint *old_comp, tirecarcass_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const springdamper_constraint *old_comp, springdamper_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const spin_angle_constraint *old_comp, spin_angle_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const spin_constraint *old_comp, spin_constraint &new_comp, merge_context &ctx) {
    merge(static_cast<const constraint_base *>(old_comp), static_cast<constraint_base &>(new_comp), ctx);
}

template<> inline
void merge(const differential_constraint *old_comp, differential_constraint &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
