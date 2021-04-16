#ifndef EDYN_DYNAMICS_ROW_CACHE_HPP
#define EDYN_DYNAMICS_ROW_CACHE_HPP

#include <vector>
#include <tuple>
#include "edyn/comp/constraint_row.hpp"

namespace edyn {

struct row_cache {

    void clear() {
        // Clear caches and keep capacity.
        con_rows.clear();
        con_num_rows.clear();
    }

    std::vector<constraint_row> con_rows;
    std::vector<size_t> con_num_rows;
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
