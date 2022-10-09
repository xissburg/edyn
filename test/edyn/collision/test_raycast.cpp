#include "../common/common.hpp"

TEST(test_raycast, raycast_box) {
    entt::registry registry;
    auto config = edyn::init_config{};
    config.execution_mode = edyn::execution_mode::sequential;
    config.num_worker_threads = 2;
    edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    def.position = {0.5, 0.5, 0.5};
    def.kind = edyn::rigidbody_kind::rb_static;
    auto box_entity = edyn::make_rigidbody(registry, def);
    edyn::update(registry);

    auto p0 = edyn::vector3{2, 2, 2};
    auto p1 = edyn::vector3{0, 0, 0};
    auto result = edyn::raycast(registry, p0, p1);

    ASSERT_EQ(result.entity, box_entity);
    ASSERT_SCALAR_EQ(result.fraction, edyn::scalar(0.5));
    ASSERT_TRUE(std::holds_alternative<edyn::box_raycast_info>(result.info_var));

    // Try another one with a vertical ray coming from above.
    p0 = {0.5, 2, 0.5};
    p1 = {0.5, 0, 0.5};
    result = edyn::raycast(registry, p0, p1);
    ASSERT_SCALAR_EQ(result.fraction, edyn::scalar(0.5));
    ASSERT_TRUE(std::holds_alternative<edyn::box_raycast_info>(result.info_var));

    auto &info = std::get<edyn::box_raycast_info>(result.info_var);
    ASSERT_EQ(info.face_index, 2);
}
