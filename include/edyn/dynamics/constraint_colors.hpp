#ifndef EDYN_DYNAMICS_CONSTRAINT_COLORS_HPP
#define EDYN_DYNAMICS_CONSTRAINT_COLORS_HPP

#include <entt/entity/sparse_set.hpp>

namespace edyn {

class constraint_colors final {
    struct entry {
        entt::sparse_set constraint;
        entt::sparse_set body;
    };

public:
    size_t insert(const entt::registry &registry, entt::entity constraint);
    void remove(const entt::registry &registry, entt::entity constraint);

    auto size() const { return m_colors.size() + (m_non_procedural.constraint.empty() ? 0 : 1); }

    template<typename Func>
    void each(Func func) {
        if (!m_non_procedural.constraint.empty()) {
            for (auto entity : m_non_procedural.constraint) {
                func(entity);
            }
        }

        for (auto &entry : m_colors) {
            for (auto entity : entry.constraint) {
                func(entity);
            }
        }
    }

    auto size(size_t index) const {
        if (index < m_colors.size()) {
            return m_colors[index].constraint.size();
        }

        if (index == m_colors.size()) {
            return m_non_procedural.constraint.size();
        }

        return SIZE_MAX;
    }

    const auto & get(size_t index) const {
        if (index < m_colors.size()) {
            return m_colors[index].constraint;
        }

        return m_non_procedural.constraint;
    }

private:
    std::vector<entry> m_colors;
    entry m_non_procedural;
};

}

#endif // EDYN_DYNAMICS_CONSTRAINT_COLORS_HPP
