#ifndef EDYN_DYNAMICS_ROW_CACHE_HPP
#define EDYN_DYNAMICS_ROW_CACHE_HPP

#include <vector>
#include <tuple>
#include "edyn/comp/constraint_row.hpp"

namespace edyn {

class solver;

class row_cache {
public:
    std::tuple<constraint_row &,constraint_row_data &> make_row() {
        auto &row = con_rows.emplace_back();
        auto &data = con_datas.emplace_back();
        return std::tie(row, data);
    }

    void clear() {
        // Clear caches and keep capacity.
        con_rows.clear();
        con_datas.clear();
        con_num_rows.clear();
    }

    std::vector<constraint_row> con_rows;
    std::vector<constraint_row_data> con_datas;
    std::vector<size_t> con_num_rows;
};

}

#endif // EDYN_DYNAMICS_ROW_CACHE_HPP
