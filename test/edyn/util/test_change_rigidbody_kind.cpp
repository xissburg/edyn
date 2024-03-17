#include "../common/common.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/rigidbody.hpp"

TEST(test_change_rigidbody_kind, change_kind_kinematic_dynamic) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    def.kind = edyn::rigidbody_kind::rb_kinematic;
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::update(registry, 0.01);

    // Must ensure non-zero mass before making it dynamic.
    edyn::set_rigidbody_mass(registry, rb, 10);
    edyn::set_rigidbody_inertia(registry, rb, edyn::matrix3x3_identity);
    edyn::rigidbody_set_kind(registry, rb, edyn::rigidbody_kind::rb_dynamic);
    edyn::update(registry, 0.01);

    ASSERT_TRUE((registry.all_of<edyn::dynamic_tag, edyn::procedural_tag>(rb)));

    edyn::detach(registry);
}

TEST(test_change_rigidbody_kind, change_kind_kinematic_dynamic_async) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::asynchronous;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    def.kind = edyn::rigidbody_kind::rb_kinematic;
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::update(registry, 0.01);

    // Must ensure non-zero mass before making it dynamic.
    edyn::set_rigidbody_mass(registry, rb, 10);
    edyn::set_rigidbody_inertia(registry, rb, edyn::matrix3x3_identity);
    edyn::rigidbody_set_kind(registry, rb, edyn::rigidbody_kind::rb_dynamic);
    edyn::update(registry, 0.01);

    ASSERT_TRUE((registry.all_of<edyn::dynamic_tag, edyn::procedural_tag>(rb)));

    edyn::detach(registry);
}

TEST(test_change_rigidbody_kind, change_kind_dynamic_kinematic) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::update(registry, 0.01);

    edyn::rigidbody_set_kind(registry, rb, edyn::rigidbody_kind::rb_kinematic);
    edyn::update(registry, 0.01);

    ASSERT_TRUE((registry.all_of<edyn::kinematic_tag>(rb)));
    ASSERT_FALSE((registry.any_of<edyn::procedural_tag>(rb)));

    edyn::detach(registry);
}

TEST(test_change_rigidbody_kind, change_kind_dynamic_kinematic_amorphous) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.kind = edyn::rigidbody_kind::rb_dynamic;
    def.mass = 1;
    def.inertia = edyn::matrix3x3_identity;
    auto rb = edyn::make_rigidbody(registry, def);
    edyn::update(registry, 0.01);

    edyn::rigidbody_set_kind(registry, rb, edyn::rigidbody_kind::rb_kinematic);
    edyn::update(registry, 0.01);

    ASSERT_TRUE((registry.all_of<edyn::kinematic_tag>(rb)));
    ASSERT_FALSE((registry.any_of<edyn::procedural_tag>(rb)));

    edyn::detach(registry);
}
