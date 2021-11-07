#include "../common/common.hpp"
#include <edyn/sys/integrate_linvel.hpp>

TEST(integrate_linvel, test) {
    entt::registry registry;
    auto e0 = registry.create();
    auto e1 = registry.create();
    auto e2 = registry.create();

    auto& p0 = registry.emplace<edyn::position>(e0, 2.f, 3.f, 4.f);
    const auto p0_0 = p0;

    auto& p1 = registry.emplace<edyn::position>(e1, -2.f, -3.f, -5.1f);
    const auto p1_0 = p1;

    registry.emplace<edyn::linvel>(e1, -0.33f, -0.5f, -0.1f);
    registry.emplace<edyn::linvel>(e2, -0.12f, -0.99f, 0.12f);

    // Only dynamic entities have their position updated.
    registry.emplace<edyn::dynamic_tag>(e0);
    registry.emplace<edyn::dynamic_tag>(e1);
    registry.emplace<edyn::dynamic_tag>(e2);

    const edyn::scalar dt = 0.1666f;
    const size_t n = 3;

    for (size_t i = 0; i < n; ++i) {
        edyn::integrate_linvel(registry, dt);
    }

    auto& p0_1 = registry.get<edyn::position>(e0);
    auto& p1_1 = registry.get<edyn::position>(e1);
    auto& v1 = registry.get<edyn::linvel>(e1);

    ASSERT_EQ(p0_1, p0_0); // e0 has no velocity, thus unchanged
    ASSERT_SCALAR_EQ(p1_1.x, (p1_0 + v1 * dt * n).x);
    ASSERT_SCALAR_EQ(p1_1.y, (p1_0 + v1 * dt * n).y);
    ASSERT_SCALAR_EQ(p1_1.z, (p1_0 + v1 * dt * n).z);
}