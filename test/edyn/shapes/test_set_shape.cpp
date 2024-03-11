#include "../common/common.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/util/rigidbody.hpp"
#include <type_traits>

TEST(test_set_shape, change_shape) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::rigidbody_set_shape(registry, rb, edyn::sphere_shape{0.5});

    ASSERT_TRUE(registry.all_of<edyn::sphere_shape>(rb));
    ASSERT_FALSE(registry.all_of<edyn::box_shape>(rb));

    edyn::visit_shape(registry, rb, [](auto &shape) {
        using ShapeType = std::decay_t<decltype(shape)>;
        ASSERT_TRUE((std::is_same_v<ShapeType, edyn::sphere_shape>));
    });

    edyn::update(registry);

    edyn::detach(registry);
}

TEST(test_set_shape, clear_shape) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto rb = edyn::make_rigidbody(registry, def);

    edyn::update(registry);

    edyn::rigidbody_set_shape(registry, rb, {});

    ASSERT_FALSE(registry.all_of<edyn::box_shape>(rb));
    ASSERT_FALSE(registry.all_of<edyn::AABB>(rb));

    edyn::detach(registry);
}

TEST(test_set_shape, from_amorphous) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.mass = 1;
    def.inertia = edyn::matrix3x3_identity;
    auto rb = edyn::make_rigidbody(registry, def);

    edyn::update(registry);

    edyn::rigidbody_set_shape(registry, rb, edyn::capsule_shape{0.1, 1});

    ASSERT_TRUE(registry.all_of<edyn::capsule_shape>(rb));
    ASSERT_TRUE(registry.all_of<edyn::AABB>(rb));

    edyn::detach(registry);
}
