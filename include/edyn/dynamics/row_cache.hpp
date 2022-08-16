#ifndef EDYN_DYNAMICS_ROW_CACHE_HPP
#define EDYN_DYNAMICS_ROW_CACHE_HPP

#include <type_traits>
#include <vector>
#include <tuple>
#include "edyn/config/config.h"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_row_friction.hpp"
#include "edyn/constraints/constraint_row_spin_friction.hpp"

namespace edyn {

static constexpr uint8_t constraint_row_flag_friction = 1;
static constexpr uint8_t constraint_row_flag_rolling_friction = 1 << 1;
static constexpr uint8_t constraint_row_flag_spinning_friction = 1 << 2;

/**
 * Stores the constraint rows for one solver update.
 */
struct row_cache {
    std::vector<constraint_row> rows;

    // Number of rows in each constraint. Values are inserted sequentially
    // for each edge in an island. One edge can have more than one constraint.
    std::vector<uint8_t> con_num_rows;

    // Bitset which stores whether the row is accompanied by friction rows.
    std::vector<uint8_t> flags;

    std::vector<constraint_row_friction> friction;
    std::vector<constraint_row_friction> rolling;
    std::vector<constraint_row_spin_friction> spinning;

    void clear() {
        rows.clear();
        con_num_rows.clear();
        flags.clear();
        friction.clear();
        rolling.clear();
        spinning.clear();
    }
};

/**
 * During constraint preparation which happens right before solving, all
 * constraint rows are inserted into this component. They are then packed
 * together into a `row_cache` for better performance during the solver
 * iterations.
 */
struct constraint_row_prep_cache {
    static constexpr unsigned max_rows = 32;
    static constexpr unsigned max_constraints = 16;

    struct element {
        constraint_row row;
        uint8_t flags; // Whether this row has friction.
        constraint_row_friction friction;
        constraint_row_friction rolling;
        constraint_row_spin_friction spinning;
        constraint_row_options options;

        void clear() {
            flags = 0;
            options = {};
        }
    };

    // All rows in this entity.
    std::array<element, max_rows> rows;
    uint8_t num_rows;

    // Number of rows per constraint in the same order they appear in the
    // `constraints_tuple`, since an entity can have multiple constraints
    // of different types.
    std::array<uint8_t, max_constraints> rows_per_constraint;
    uint8_t num_constraints;

    constraint_row_prep_cache() {
        clear();
    }

    void add_constraint() {
        ++num_constraints;
    }

    constraint_row & add_row() {
        EDYN_ASSERT(num_rows < max_rows);
        EDYN_ASSERT(num_constraints > 0);
        ++rows_per_constraint[num_constraints - 1];
        auto &elem = rows[num_rows++];
        elem.flags = 0;
        return elem.row;
    }

    constraint_row_friction & add_friction_row() {
        EDYN_ASSERT(num_constraints > 0);
        auto &curr_row = rows[num_rows - 1];
        EDYN_ASSERT(!(curr_row.flags & constraint_row_flag_friction));
        curr_row.flags |= constraint_row_flag_friction;
        return curr_row.friction;
    }

    constraint_row_friction & add_rolling_row() {
        EDYN_ASSERT(num_constraints > 0);
        auto &curr_row = rows[num_rows - 1];
        EDYN_ASSERT(!(curr_row.flags & constraint_row_flag_rolling_friction));
        curr_row.flags |= constraint_row_flag_rolling_friction;
        return curr_row.rolling;
    }

    constraint_row_spin_friction & add_spinning_row() {
        EDYN_ASSERT(num_constraints > 0);
        auto &curr_row = rows[num_rows - 1];
        EDYN_ASSERT(!(curr_row.flags & constraint_row_flag_spinning_friction));
        curr_row.flags |= constraint_row_flag_spinning_friction;
        return curr_row.spinning;
    }

    // Get preparation options for the current row.
    constraint_row_options & get_options() {
        EDYN_ASSERT(num_constraints > 0);
        auto &curr_row = rows[num_rows - 1];
        return curr_row.options;
    }

    void clear() {
        num_rows = 0;
        num_constraints = 0;

        for (auto &elem : rows) {
            elem.clear();
        }

        for (auto &c : rows_per_constraint) {
            c = 0;
        }
    }
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
