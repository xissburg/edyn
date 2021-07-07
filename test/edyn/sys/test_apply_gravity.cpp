#include "../common/common.hpp"
#include <edyn/sys/apply_gravity.hpp>

TEST(apply_gravity, test) {
    entt::registry registry;
    auto ent = registry.create();
    registry.emplace<edyn::gravity>(ent, edyn::gravity_earth);
    registry.emplace<edyn::linvel>(ent, edyn::vector3_zero);

    // Only dynamic entities have their velocity updated.
    registry.emplace<edyn::dynamic_tag>(ent);

    const edyn::scalar dt = 0.1666;
    const size_t n = 10;

    for (size_t i = 0; i < n; ++i) {
        edyn::apply_gravity(registry, dt);
    }

    auto& linvel = registry.get<edyn::linvel>(ent);
    ASSERT_SCALAR_EQ(linvel.x, (edyn::gravity_earth * dt * n).x);
    ASSERT_SCALAR_EQ(linvel.y, (edyn::gravity_earth * dt * n).y);
    ASSERT_SCALAR_EQ(linvel.z, (edyn::gravity_earth * dt * n).z);
}