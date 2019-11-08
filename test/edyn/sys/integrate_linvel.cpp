#include "../common/common.hpp"
#include <edyn/sys/integrate_linvel.hpp>

TEST(integrate_linvel, test) {
    entt::registry registry;
    auto e0 = registry.create();
    auto e1 = registry.create();
    auto e2 = registry.create();

    auto& p0 = registry.assign<edyn::position>(e0, 2, 3, 4);
    const auto p0_0 = p0;

    auto& p1 = registry.assign<edyn::position>(e1, -2, -3, -5.1);
    const auto p1_0 = p1;
    
    registry.assign<edyn::linvel>(e1, -0.33, -0.5, -0.1);
    registry.assign<edyn::linvel>(e2, -0.12, -0.99, 0.12);

    const edyn::scalar dt = 0.1666;
    const size_t n = 3;

    for (size_t i = 0; i < n; ++i) {
        edyn::integrate_linvel(registry, dt);
    }

    auto& p0_1 = registry.get<edyn::position>(e0);
    auto& p1_1 = registry.get<edyn::position>(e1);
    auto& v1 = registry.get<edyn::linvel>(e1);

    ASSERT_EQ(p0_1, p0_0); // e0 has no velocity, thus unchanged
    ASSERT_DOUBLE_EQ(p1_1.x, (p1_0 + v1 * dt * n).x);
    ASSERT_DOUBLE_EQ(p1_1.y, (p1_0 + v1 * dt * n).y);
    ASSERT_DOUBLE_EQ(p1_1.z, (p1_0 + v1 * dt * n).z);
}