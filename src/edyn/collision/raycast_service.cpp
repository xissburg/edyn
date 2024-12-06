#include "edyn/collision/raycast_service.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/context/task.hpp"
#include "edyn/context/task_util.hpp"
#include "edyn/util/vector_util.hpp"
#include <entt/signal/delegate.hpp>

namespace edyn {

raycast_service::raycast_service(entt::registry &registry)
    : m_registry(&registry)
{}

void raycast_service::run_broadphase(bool mt) {
    auto &bphase = m_registry->ctx().get<broadphase>();

    if (mt && m_broad_ctx.size() > m_max_raycast_broadphase_sequential_size) {
        auto *raycasts = &m_broad_ctx;

        auto task_func = [raycasts, &bphase](unsigned start, unsigned end) {
            for (auto index = start; index < end; ++index) {
                auto &ctx = (*raycasts)[index];
                bphase.raycast(ctx.p0, ctx.p1, [&](entt::entity entity) {
                    if (!vector_contains(ctx.ignore_entities, entity)) {
                        ctx.candidates.push_back(entity);
                    }
                });
            }
        };

        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(*m_registry, task, m_broad_ctx.size());
    } else {
        for (auto &ctx : m_broad_ctx) {
            bphase.raycast(ctx.p0, ctx.p1, [&](entt::entity entity) {
                if (!vector_contains(ctx.ignore_entities, entity)) {
                    ctx.candidates.push_back(entity);
                }
            });
        }
    }
}

void raycast_service::finish_broadphase() {
    for (auto &ctx : m_broad_ctx) {
        for (auto entity : ctx.candidates) {
            auto &narrow_ctx = m_narrow_ctx.emplace_back();
            narrow_ctx.id = ctx.id;
            narrow_ctx.p0 = ctx.p0;
            narrow_ctx.p1 = ctx.p1;
            narrow_ctx.entity = entity;
        }
    }

    m_broad_ctx.clear();
}

void raycast_service::run_narrowphase(bool mt) {
    auto index_view = m_registry->view<shape_index>();
    auto tr_view = m_registry->view<position, orientation>();
    auto origin_view = m_registry->view<origin>();
    auto shape_views_tuple = get_tuple_of_shape_views(*m_registry);

    if (mt && m_narrow_ctx.size() > m_max_raycast_narrowphase_sequential_size) {
        auto *ctxes = &m_narrow_ctx;

        auto task_func = [ctxes, index_view, origin_view, tr_view, shape_views_tuple](unsigned start, unsigned end) {
            for (auto index = start; index < end; ++index) {
                auto &ctx = (*ctxes)[index];

                auto sh_idx = index_view.get<shape_index>(ctx.entity);
                auto pos = origin_view.contains(ctx.entity) ?
                    static_cast<vector3>(origin_view.get<origin>(ctx.entity)) : tr_view.get<position>(ctx.entity);
                auto orn = tr_view.get<orientation>(ctx.entity);
                auto ray_ctx = raycast_context{pos, orn, ctx.p0, ctx.p1};

                visit_shape(sh_idx, ctx.entity, shape_views_tuple, [&](auto &&shape) {
                    ctx.result = shape_raycast(shape, ray_ctx);
                });
            }
        };

        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(*m_registry, task, m_narrow_ctx.size());
    } else {
        auto index_view = m_registry->view<shape_index>();
        auto tr_view = m_registry->view<position, orientation>();
        auto origin_view = m_registry->view<origin>();
        auto shape_views_tuple = get_tuple_of_shape_views(*m_registry);

        for (auto &ctx : m_narrow_ctx) {
            auto sh_idx = index_view.get<shape_index>(ctx.entity);
            auto pos = origin_view.contains(ctx.entity) ?
                static_cast<vector3>(origin_view.get<origin>(ctx.entity)) : tr_view.get<position>(ctx.entity);
            auto orn = tr_view.get<orientation>(ctx.entity);
            auto ray_ctx = raycast_context{pos, orn, ctx.p0, ctx.p1};

            visit_shape(sh_idx, ctx.entity, shape_views_tuple, [&](auto &&shape) {
                ctx.result = shape_raycast(shape, ray_ctx);
            });
        }
    }
}

void raycast_service::finish_narrowphase() {
    for (auto &ctx : m_narrow_ctx) {
        auto &res = m_results[ctx.id];

        if (ctx.result.fraction < res.fraction) {
            res = ctx.result;
            res.entity = ctx.entity;
        }
    }

    m_narrow_ctx.clear();
}

void raycast_service::update(bool mt) {
    run_broadphase(mt);
    finish_broadphase();
    run_narrowphase(mt);
    finish_narrowphase();
}

}
