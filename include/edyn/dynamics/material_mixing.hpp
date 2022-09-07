#ifndef EDYN_DYNAMICS_MATERIAL_MIXING_HPP
#define EDYN_DYNAMICS_MATERIAL_MIXING_HPP

#include <limits>
#include <map>
#include <utility>
#include "edyn/core/unordered_pair.hpp"
#include "edyn/comp/material.hpp"

namespace edyn {

inline scalar material_mix_restitution(scalar a, scalar b) {
    return std::min(a, b);
}

inline scalar material_mix_friction(scalar a, scalar b) {
    return std::sqrt(a * b);
}

inline scalar material_mix_spin_friction(scalar a, scalar b) {
    return std::max(a, b);
}

inline scalar material_mix_roll_friction(scalar a, scalar b) {
    return std::max(a, b);
}

inline scalar material_mix_stiffness(scalar a, scalar b) {
    return 1 / (1 / a + 1 / b);
}

inline scalar material_mix_damping(scalar a, scalar b) {
    return 1 / (1 / a + 1 / b);
}

class material_mix_table {
public:
    using pair_type = unordered_pair<material::id_type>;

    bool contains(const pair_type &pair) const {
        return m_table.count(pair) != 0;
    }

    void insert(const pair_type &pair, const material_base &material) {
    #ifndef EDYN_DISABLE_ASSERT
        EDYN_ASSERT(pair.first != std::numeric_limits<material::id_type>::max());
        EDYN_ASSERT(pair.second != std::numeric_limits<material::id_type>::max());
    #endif
        m_table[pair] = material;
    }

    material_base & get(const pair_type &pair) {
        return m_table.at(pair);
    }

    const material_base & get(const pair_type &pair) const {
        return m_table.at(pair);
    }

    const material_base * try_get(const pair_type &pair) const {
        if (m_table.count(pair)) {
            return &m_table.at(pair);
        }
        return nullptr;
    }

    material_base * try_get(const pair_type &pair) {
        return const_cast<material_base *>(std::as_const(*this).try_get(pair));
    }

    void remove(const pair_type &pair) {
        m_table.erase(pair);
    }

private:
    std::map<pair_type, material_base> m_table;
};

}

#endif // EDYN_DYNAMICS_MATERIAL_MIXING_HPP
