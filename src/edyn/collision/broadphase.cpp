#include "edyn/collision/broadphase.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>
#include <vector>

namespace edyn {

bool intersect(const AABB &b0, const AABB &b1) {
    return (b0.min.x <= b1.max.x) &&
		   (b0.max.x >= b1.min.x) &&
		   (b0.min.y <= b1.max.y) &&
		   (b0.max.y >= b1.min.y) &&
		   (b0.min.z <= b1.max.z) &&
		   (b0.max.z >= b1.min.z);
}

broadphase::broadphase(entt::registry &reg)
    : registry(&reg)
{

}

void broadphase::update() {
    auto view = registry->view<const position, const orientation, const shape, AABB>();
    view.each([] (auto, auto &pos, auto &orn, auto &sh, auto &aabb) {
        std::visit([&] (auto &&s) {
            aabb = s.aabb(pos, orn);
        }, sh.var);
    });

    auto aabb_view = registry->view<const AABB>();
    auto rel_view = registry->view<const relation>();

    // Destroy relations that are not intersecting anymore.
    std::vector<entt::entity> destroy_rel;
    rel_view.each([&] (auto ent, auto &rel) {
        auto b0 = registry->try_get<AABB>(rel.entity[0]);
        auto b1 = registry->try_get<AABB>(rel.entity[1]);

        if (b0 && b1 && !intersect(*b0, *b1)) {
            destroy_rel.push_back(ent);
            auto p = std::make_pair(rel.entity[0], rel.entity[1]);
            auto q = std::make_pair(rel.entity[1], rel.entity[0]);
            relations.erase(p);
            relations.erase(q);
        }
    });

    for (auto ent : destroy_rel) {
        registry->destroy(ent);
    }

    // Search for new AABB intersections and create contact constraints.
    auto it = aabb_view.begin();
    const auto it_end = aabb_view.end();

    for (; it != it_end; ++it) {
        auto e0 = *it;
        auto &b0 = aabb_view.get(e0);

        for (auto it1 = it + 1; it1 != it_end; ++it1) {
            auto e1 = *it1;
            auto &b1 = aabb_view.get(e1);

            if (intersect(b0, b1)) {
                auto p = std::make_pair(e0, e1);
                if (!relations.count(p)) {
                    auto ent = make_constraint(*registry, contact_constraint(), e0, e1);
                    relations[p] = ent;
                    // Also store the reverse pair.
                    auto q = std::make_pair(e1, e0);
                    relations[q] = ent;
                }
            }
        }
    }
}

}