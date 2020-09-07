#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

void print_entities(entt::registry& registry, edyn::scalar dt) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();   
    auto time = step * world.fixed_dt;

    printf("================================\n");
    printf("step %lu, dt %.6f, time %.2f\n", step, dt, time);

    auto view = registry.view<const edyn::position, const edyn::linvel>();
    view.each([] (auto ent, const auto &pos, const auto &vel) {
        printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        printf("vel (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), vel.x, vel.y, vel.z);
    });

    if (time >= 10) {
        world.quit();
    }                                                                                                                                
}

int main(int argc, char** argv) {
    entt::registry registry;
    // Create an `edyn::world` into the registry's context. THe `edyn::world`
    // must be created before any rigid bodies are added to the registry.
    auto& world = registry.set<edyn::world>(registry);
    world.update_sink().connect<&print_entities>(registry);

    edyn::job_dispatcher::global().start();

    auto def = edyn::rigidbody_def();
    def.presentation = true;
    def.friction = 0.8;
    def.mass = 10;
    def.restitution = 0;
    def.position = {0, 3, 0};
    def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.7);
    def.shape_opt = {edyn::cylinder_shape{0.2, 0.5}};
    def.update_inertia();
    edyn::make_rigidbody(registry, def);

    // Run physics main loop. Could also simply call `world.update(dt)` in
    // your own main loop.
    world.run();

    return 0;
}