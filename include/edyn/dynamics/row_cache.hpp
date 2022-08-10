#ifndef EDYN_DYNAMICS_ROW_CACHE_HPP
#define EDYN_DYNAMICS_ROW_CACHE_HPP

#include <vector>
#include <tuple>
#include "edyn/config/config.h"
#include "edyn/constraints/constraint_row.hpp"

namespace edyn {

/**
 * Stores the constraint rows for one solver update.
 */
struct row_cache {

    void clear() {
        // Clear caches and keep capacity.
        rows.clear();
        con_num_rows.clear();
    }

    std::vector<constraint_row> rows;

    // Number of rows in each constraint. This is sorted in the same order
    // as in the pool of each constraint type and ordered by the order which
    // the constraint types appear in the `constraints_tuple`.
    std::vector<size_t> con_num_rows;
};

struct constraint_row_prep_cache {
    static constexpr size_t max_rows = 32;
    std::array<constraint_row, max_rows> rows;
    uint8_t num_rows;
    std::array<uint8_t, 16> rows_per_constraint;
    uint8_t num_constraints;

    constraint_row_prep_cache() {
        clear();
    }

    constraint_row & add_row() {
        EDYN_ASSERT(num_rows < max_rows);
        EDYN_ASSERT(num_constraints > 0);
        ++rows_per_constraint[num_constraints - 1];
        return rows[num_rows++];
    }

    void add_constraint() {
        ++num_constraints;
    }

    void clear() {
        num_rows = 0;
        num_constraints = 0;

        for (auto &c : rows_per_constraint) {
            c = 0;
        }
    }
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
