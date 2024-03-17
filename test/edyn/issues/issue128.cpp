#include "../common/common.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/edyn.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/rigidbody.hpp"

TEST(issue_test, test_issue_76) {
	// https://github.com/xissburg/edyn/issues/128
	entt::registry registry;

	auto config = edyn::init_config{};
	config.execution_mode = edyn::execution_mode::sequential;
	edyn::attach(registry, config);

    auto def = edyn::rigidbody_def{};
    def.shape = edyn::box_shape{0.5, 0.5, 0.5};
    auto rbA = edyn::make_rigidbody(registry, def);

    def.position = {0, 2, 0};
    auto rbB = edyn::make_rigidbody(registry, def);

    const auto setup_con =
        [](edyn::distance_constraint & con) {
            con.distance = 2;
            con.pivot[0] = edyn::vector3_zero;
            con.pivot[1] = edyn::vector3_zero;
        };
    auto con = edyn::make_constraint<edyn::distance_constraint>(registry, rbA, rbB, setup_con);

    edyn::update(registry);

    edyn::detach(registry);

    ASSERT_TRUE(registry.valid(rbA));
    ASSERT_TRUE(registry.valid(rbB));
    ASSERT_TRUE(registry.valid(con));

    ASSERT_FALSE(edyn::validate_rigidbody(registry, rbA));
    ASSERT_FALSE(edyn::validate_rigidbody(registry, rbB));
    ASSERT_FALSE((registry.all_of<edyn::distance_constraint>(con)));

	edyn::attach(registry, config);
    edyn::make_rigidbody(rbA, registry, def);
    edyn::make_rigidbody(rbB, registry, def);
    edyn::make_constraint<edyn::distance_constraint>(registry, con, rbA, rbB, setup_con);

    ASSERT_TRUE(edyn::validate_rigidbody(registry, rbA));
    ASSERT_TRUE(edyn::validate_rigidbody(registry, rbB));

    edyn::update(registry);

    edyn::detach(registry);

    ASSERT_TRUE(registry.valid(rbA));
    ASSERT_TRUE(registry.valid(rbB));
    ASSERT_TRUE(registry.valid(con));

    ASSERT_FALSE(edyn::validate_rigidbody(registry, rbA));
    ASSERT_FALSE(edyn::validate_rigidbody(registry, rbB));
}
