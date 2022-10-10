#include "edyn/math/matrix3x3.hpp"
#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

/**
 * Demonstration of the importance of interpolation for smooth real-time
 * presentation on screen using `edyn::present_position`. Using the raw
 * `edyn::position` will generally cause jitter. The value in
 * `edyn::present_position` is the `edyn::position` interpolated or
 * extrapolated by an amount of time using the `edyn::linvel` of the entity.
 */
void print_entities(entt::registry& registry) {
    printf("================================\n");

    auto view = registry.view<const edyn::position, const edyn::present_position>();
    view.each([](auto ent, const auto& pos, const auto& curpos) {
        // Compare the physics position to the presentation position and notice how
        // the physics position `pos` does not change uniformly after each update.
        // The presentation position `curpos` should change linearly with dt (because
        // `linvel` is constant in this example).
        printf("pos    (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), pos.x, pos.y, pos.z);
        printf("curpos (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), curpos.x, curpos.y, curpos.z);
    });
}

int main(int argc, char** argv) {
    entt::registry registry;
    edyn::attach(registry);
    edyn::set_fixed_dt(registry, 0.041);

    auto def = edyn::rigidbody_def();
    def.presentation = true;
    def.mass = 10;
    def.inertia = edyn::matrix3x3_identity;
    def.linvel = {0, 1/0.041, 0};
    def.gravity = edyn::vector3_zero;
    edyn::make_rigidbody(registry, def);

    for (;;) {
        edyn::update(registry);
        print_entities(registry);
        edyn::delay(100);
    }

    edyn::detach(registry);

    return 0;
}
