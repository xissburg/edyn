#include "edyn/dynamics/constraint_colors.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/constraint_util.hpp"
#include <cstdint>
#include <entt/entity/registry.hpp>

namespace edyn {

size_t constraint_colors::insert(const entt::registry &registry, entt::entity constraint) {
    auto bodies = constraint_get_bodies(registry, constraint);
    auto proc_view = registry.view<procedural_tag>();

    if (!proc_view.contains(bodies.first) || !proc_view.contains(bodies.second)) {
        m_non_procedural.constraint.push(constraint);
        m_non_procedural.body.push(proc_view->contains(bodies.first) ? bodies.first : bodies.second);
        return SIZE_MAX;
    }

    for (size_t i = 0; i < m_colors.size(); ++i) {
        auto &color = m_colors[i];

        if (!color.body.contains(bodies.first) && !color.body.contains(bodies.second)) {
            color.body.push(bodies.first);
            color.body.push(bodies.second);
            color.constraint.push(constraint);
            return i;
        }
    }

    auto &color = m_colors.emplace_back();
    color.body.push(bodies.first);
    color.body.push(bodies.second);
    color.constraint.push(constraint);

    return m_colors.size() - 1;
}

void constraint_colors::remove(const entt::registry &registry, entt::entity constraint) {
    auto bodies = constraint_get_bodies(registry, constraint);

    m_non_procedural.body.remove(bodies.first);
    m_non_procedural.body.remove(bodies.second);
    m_non_procedural.constraint.remove(constraint);

    for (auto &color : m_colors) {
        if (color.constraint.contains(constraint)) {
            color.constraint.erase(constraint);
            color.body.erase(bodies.first);
            color.body.erase(bodies.second);
        }
    }
}

}
