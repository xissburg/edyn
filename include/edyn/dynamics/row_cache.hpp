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
    template<typename T>
    struct row_container {
        std::vector<T> rows;
        // Number of rows in each constraint. This is sorted in the same order
        // as in the pool of each constraint type and ordered by the order which
        // the constraint types appear in the `constraints_tuple`.
        std::vector<uint8_t> con_num_rows;

        void clear() {
            // Clear caches and keep capacity.
            rows.clear();
            con_num_rows.clear();
        }
    };

    void clear() {
        regular.clear();
        friction.clear();
    }

    row_container<constraint_row> regular;
    row_container<constraint_row_friction> friction;
};

struct constraint_row_prep_cache {
    static constexpr unsigned max_rows = 32;
    static constexpr unsigned max_constraints = 16;

    template<typename T>
    struct row_container {
        std::array<T, max_rows> rows;
        uint8_t num_rows;
        std::array<uint8_t, max_constraints> rows_per_constraint;

        T & add_row(uint8_t constraint_index) {
            EDYN_ASSERT(constraint_index < max_constraints);
            ++rows_per_constraint[constraint_index];
            EDYN_ASSERT(num_rows < max_rows);
            return rows[num_rows++];
        }

        void clear() {
            num_rows = 0;

            for (auto &c : rows_per_constraint) {
                c = 0;
            }
        }
    };

    row_container<constraint_row> regular;
    row_container<constraint_row_friction> friction;
    uint8_t num_constraints;

    constraint_row_prep_cache() {
        clear();
    }

    constraint_row & add_row() {
        EDYN_ASSERT(num_constraints > 0);
        return regular.add_row(num_constraints - 1);
    }

    constraint_row_friction & add_friction_row() {
        EDYN_ASSERT(num_constraints > 0);
        return friction.add_row(num_constraints - 1);
    }

    void add_constraint() {
        ++num_constraints;
    }

    void clear() {
        regular.clear();
        friction.clear();
        num_constraints = 0;
    }
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
