#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

void print_entities(entt::registry& registry, edyn::scalar dt) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();
    
    printf("===============================\n");
    printf("step %lu, dt %.6f\n", step, dt);

    auto view = registry.view<const edyn::current_position>();
    view.each([] (auto ent, const auto& pos) {
        printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
    });

    if (step * world.fixed_dt > 10) {
        world.quit();
    }
}

int main(int argc, char** argv) {
    entt::registry registry;

    const auto ent = registry.create();
    registry.assign<edyn::position>(ent, 0, 3, 0);
    registry.assign<edyn::current_position>(ent);
    registry.assign<edyn::linvel>(ent);
    registry.assign<edyn::linacc>(ent, edyn::gravity_earth);

    // Create an `edyn::world` into the registry's context.
    auto& world = registry.set<edyn::world>(registry);
    world.update_sink().connect<&print_entities>(registry);
    // Run physics main loop. Could also simply call `world.update(dt)` in
    // your own main loop.
    world.run();

    return 0;
}