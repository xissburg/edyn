#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

void print_entities(entt::registry& registry) {
    printf("================================\n");

    auto view = registry.view<const edyn::position, const edyn::linvel>();
    view.each([](auto ent, const auto &pos, const auto &vel) {
        printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), pos.x, pos.y, pos.z);
        printf("vel (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), vel.x, vel.y, vel.z);
    });
}

int main(int argc, char** argv) {
    entt::registry registry;
    edyn::attach(registry);

    auto def = edyn::rigidbody_def();
    def.presentation = true;
    def.mass = 10;
    def.material->friction = 0.8;
    def.material->restitution = 0;
    def.position = {0, 3, 0};
    def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.7);
    def.shape = edyn::cylinder_shape{0.2, 0.5};
    edyn::make_rigidbody(registry, def);

    for (;;) {
        edyn::update(registry);
        print_entities(registry);
        edyn::delay(100);
    }

    edyn::detach(registry);

    return 0;
}
