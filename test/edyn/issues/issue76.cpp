#include "../common/common.hpp"
#include "edyn/edyn.hpp"

TEST(issue_test, test_issue_76) {
	// https://github.com/xissburg/edyn/issues/76
	entt::registry registry;

	edyn::init({2});
	edyn::attach(registry);

	// Create floor
	auto floor_def = edyn::rigidbody_def();
	floor_def.kind = edyn::rigidbody_kind::rb_static;
	floor_def.shape = edyn::plane_shape{ {0, 1, 0}, 0 };
	auto entity = edyn::make_rigidbody(registry, floor_def);

	registry.destroy(entity);

	auto entity2 = edyn::make_rigidbody(registry, floor_def);
	edyn::update(registry);
	edyn::detach(registry);
	edyn::deinit();
}
