#include "../common/common.hpp"
#include "edyn/util/rigidbody.hpp"

TEST(test_clear_rigidbody, clear_rigidbody) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::clear_rigidbody(registry, rb);

    ASSERT_TRUE(registry.valid(rb));
    ASSERT_FALSE(edyn::validate_rigidbody(registry, rb));
    ASSERT_FALSE((registry.any_of<edyn::position, edyn::linvel, edyn::mass>(rb)));

    edyn::update(registry);
    edyn::make_rigidbody(rb, registry, def);

    edyn::update(registry);

    ASSERT_TRUE(edyn::validate_rigidbody(registry, rb));
    edyn::clear_rigidbody(registry, rb);

    ASSERT_TRUE(registry.valid(rb));
    ASSERT_FALSE(edyn::validate_rigidbody(registry, rb));
    ASSERT_FALSE((registry.any_of<edyn::orientation, edyn::angvel, edyn::inertia>(rb)));

    registry.destroy(rb);

    edyn::detach(registry);
}
