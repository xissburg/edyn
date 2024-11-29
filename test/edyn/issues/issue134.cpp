#include "../common/common.hpp"
#include "edyn/collision/broadphase.hpp"

TEST(issue_test, test_issue_134) {
	// https://github.com/xissburg/edyn/issues/134
	entt::registry registry;

	auto config = edyn::init_config{};
	edyn::attach(registry, config);
	edyn::detach(registry);

    ASSERT_FALSE(registry.ctx().contains<edyn::broadphase>());
}
