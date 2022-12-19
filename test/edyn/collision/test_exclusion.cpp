#include "../common/common.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/util/exclude_collision.hpp"

TEST(test_collision_exclusion, collision_exclusion) {
    entt::registry registry;

    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def();
    def.shape = edyn::box_shape{0.2, 0.2, 0.2};

    auto e0 = edyn::make_rigidbody(registry, def);
    auto e1 = edyn::make_rigidbody(registry, def);
    auto e2 = edyn::make_rigidbody(registry, def);

    edyn::exclude_collision(registry, e0, e1);
    edyn::exclude_collision(registry, e0, e2);

    {
        auto &excl0 = registry.get<edyn::collision_exclusion>(e0);
        ASSERT_TRUE(excl0.num_entities() == 2);
        ASSERT_TRUE(excl0.entity[0] == e1);
        ASSERT_TRUE(excl0.entity[1] == e2);

        auto &excl1 = registry.get<edyn::collision_exclusion>(e1);
        ASSERT_TRUE(excl1.num_entities() == 1);
        ASSERT_TRUE(excl1.entity[0] == e0);

        auto &excl2 = registry.get<edyn::collision_exclusion>(e2);
        ASSERT_TRUE(excl1.num_entities() == 1);
        ASSERT_TRUE(excl1.entity[0] == e0);
    }

    edyn::remove_collision_exclusion(registry, e1, e0);

    {
        // Use scope to prevent accessing references to destroyed instances.
        auto &excl0 = registry.get<edyn::collision_exclusion>(e0);
        auto &excl1 = registry.get<edyn::collision_exclusion>(e1);
        ASSERT_TRUE(excl0.num_entities() == 1);
        ASSERT_TRUE(excl0.entity[0] == e2);
        ASSERT_TRUE(excl1.num_entities() == 0);
    }

    edyn::clear_collision_exclusion(registry, e0);

    {
        auto &excl2 = registry.get<edyn::collision_exclusion>(e2);
        ASSERT_TRUE(excl2.num_entities() == 0);
    }

    edyn::detach(registry);
}
