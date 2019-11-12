#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>
#include <thread>

void print_entities(entt::registry& registry) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();   
    auto time = step * world.fixed_dt;

    printf("===============================\n");
    printf("step %lu, time %.2f\n", step, time);

    auto view = registry.view<const edyn::current_position, const edyn::linvel>();
    view.each([] (auto ent, const auto &pos, const auto &vel) {
        printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        printf("vel (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), vel.x, vel.y, vel.z);
    });

    if (time >= 10) {
        world.quit();
    }
}

void create_stuff(entt::registry& registry) {
    const auto ent = registry.create();
    // This entity has aposition in space.
    registry.assign<edyn::position>(ent, 0, 3, 0);
    // Current position used for presentation. See `current_pos.cpp` for details.
    registry.assign<edyn::current_position>(ent);
    // Linear velocity.
    registry.assign<edyn::linvel>(ent, 0, 10, 0);
    // Gravity linear acceleration.
    registry.assign<edyn::linacc>(ent, edyn::gravity_earth);
}

int main(int argc, char** argv) {
    entt::registry registry;
    auto& world = registry.set<edyn::world>(registry);
    auto thrd = std::thread(&edyn::world::run, &world);

    world.defer<&create_stuff>({});

    for (int i = 0; i < 100; ++i) {
        world.update_current_positions();
        print_entities(registry);
        edyn::delay(100);
    }

    world.quit();
    thrd.join();

    return 0;
}