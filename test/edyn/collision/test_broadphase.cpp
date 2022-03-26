#include "../common/common.hpp"

TEST(test_broadphase, collision_filtering) {
    entt::registry registry;
    edyn::init({2});
    edyn::attach(registry);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto first = edyn::make_rigidbody(registry, def);
    auto second = edyn::make_rigidbody(registry, def);
    auto third = edyn::make_rigidbody(registry, def);

    ASSERT_TRUE(edyn::should_collide_default(registry, first, second));
    ASSERT_TRUE(edyn::should_collide_default(registry, first, third));
    ASSERT_TRUE(edyn::should_collide_default(registry, second, third));

    registry.get<edyn::collision_filter>(first).group = 0x1;
    registry.get<edyn::collision_filter>(first).mask = ~0x2;

    registry.get<edyn::collision_filter>(second).group = 0x2;
    registry.get<edyn::collision_filter>(second).mask = ~0x1;

    ASSERT_FALSE(edyn::should_collide_default(registry, first, second));
    ASSERT_TRUE(edyn::should_collide_default(registry, first, third));
    ASSERT_TRUE(edyn::should_collide_default(registry, second, third));

    edyn::exclude_collision(registry, first, third);

    ASSERT_FALSE(edyn::should_collide_default(registry, first, second));
    ASSERT_FALSE(edyn::should_collide_default(registry, first, third));
    ASSERT_TRUE(edyn::should_collide_default(registry, second, third));

    edyn::detach(registry);
    edyn::deinit();
}
