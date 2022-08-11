#ifndef EDYN_DYNAMICS_ROW_CACHE_HPP
#define EDYN_DYNAMICS_ROW_CACHE_HPP

#include <vector>
#include <tuple>
#include "edyn/config/config.h"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_row_friction.hpp"

namespace edyn {

/**
 * Stores the constraint rows for one solver update.
 */
struct row_cache {

    void clear() {
        // Clear caches and keep capacity.
        rows.clear();
        con_num_rows.clear();
        friction_rows.clear();
        con_num_friction_rows.clear();
    }

    std::vector<constraint_row> rows;

    // Number of rows in each constraint. This is sorted in the same order
    // as in the pool of each constraint type and ordered by the order which
    // the constraint types appear in the `constraints_tuple`.
    std::vector<uint8_t> con_num_rows;

    std::vector<constraint_row_friction> friction_rows;
    std::vector<uint8_t> con_num_friction_rows;
};

struct constraint_row_prep_cache {
    static constexpr unsigned max_rows = 32;
    std::array<constraint_row, max_rows> rows;
    uint8_t num_rows;
    std::array<uint8_t, 16> rows_per_constraint;
    uint8_t num_constraints;

    std::array<constraint_row_friction, max_rows> friction_rows;
    uint8_t num_friction_rows;
    std::array<uint8_t, 16> friction_rows_per_constraint;

    constraint_row_prep_cache() {
        clear();
    }

    constraint_row & add_row() {
        EDYN_ASSERT(num_rows < max_rows);
        EDYN_ASSERT(num_constraints > 0);
        ++rows_per_constraint[num_constraints - 1];
        return rows[num_rows++];
    }

    constraint_row_friction & add_friction_row() {
        EDYN_ASSERT(num_friction_rows < max_rows);
        EDYN_ASSERT(num_constraints > 0);
        ++friction_rows_per_constraint[num_constraints - 1];
        return friction_rows[num_friction_rows++];
    }

    void add_constraint() {
        ++num_constraints;
    }

    void clear() {
        num_rows = 0;
        num_friction_rows = 0;
        num_constraints = 0;

        for (auto &c : rows_per_constraint) {
            c = 0;
        }

        for (auto &c : friction_rows_per_constraint) {
            c = 0;
        }
    }
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
