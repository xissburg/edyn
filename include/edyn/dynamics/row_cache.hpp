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

struct row_cache_sparse {
    static constexpr size_t max_rows = 32;
    struct entry {
        std::array<constraint_row, max_rows> rows;
        size_t count {};

        constraint_row & add_row() {
            EDYN_ASSERT(count < max_rows);
            return rows[count++];
        }
    };

    std::vector<entry> entries;
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
