#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

/**
 * Demonstration of the importance of interpolation for smooth real-time
 * presentation on screen using `edyn::current_position`. Using the raw
 * `edyn::position` will generally cause jitter.
 */
void print_entities(entt::registry& registry, edyn::scalar dt) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();

    printf("===============================\n");
    printf("step %lu, dt %.6f\n", world.current_step(), dt);

    auto view = registry.view<const edyn::position, const edyn::current_position>();
    view.each([] (auto ent, const auto& pos, const auto& curpos) {
        // Compare the physics position to the presentation positon and notice how
        // the physics position `pos` does not change uniformly after each update because
        // dt is not necesarily a multiple of fixed_dt, thus leaving a residual. The
        // presentation position `curpos` should change 1 unit per update (it also grows 
        // a thousandth every once in while since dt is not exactly the same every time).
        printf("pos    (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        printf("curpos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), curpos.x, curpos.y, curpos.z);
    });

    if (step * world.fixed_dt > 10) {
        world.quit();
    }
}

int main(int argc, char** argv) {
    entt::registry registry;

    const auto ent = registry.create();
    registry.assign<edyn::position>(ent, 0, 0, 0);
    registry.assign<edyn::current_position>(ent);
    // Set a speed that will move the entity 1 unit per step.
    registry.assign<edyn::linvel>(ent, 0, 1/0.041, 0);

    auto& world = registry.set<edyn::world>(registry);
    world.fixed_dt = 0.041;
    world.update_sink().connect<&print_entities>(registry);
    world.run();

    return 0;
}