#include "edyn/collision/raycast_service.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/util/vector_util.hpp"

namespace edyn {

raycast_service::raycast_service(entt::registry &registry)
    : m_registry(&registry)
{}

bool raycast_service::run_state_machine(job &completion_job) {
    switch (m_state) {
    case state::begin:
        if (run_broadphase_async(completion_job)) {
            return run_state_machine(completion_job);
        } else {
            // Broadphase running asynchronously.
            return false;
        }
        break;
    case state::broadphase:
        finish_broadphase();

        if (run_narrowphase_async(completion_job)) {
            return run_state_machine(completion_job);
        } else {
            // Narrowphase running asynchronously.
            return false;
        }
        break;
    case state::narrowphase:
        finish_narrowphase();
        return run_state_machine(completion_job);
        break;
    case state::done:
        m_state = state::begin;
        return true;
        break;
    }
}

bool raycast_service::update_async(job &completion_job) {
    return run_state_machine(completion_job);
}

void raycast_service::run_broadphase() {
    auto &bphase = m_registry->ctx().at<broadphase>();

    for (auto &ctx : m_broad_ctx) {
        bphase.raycast(ctx.p0, ctx.p1, [&](entt::entity entity) {
            ctx.candidates.push_back(entity);
        });
    }
}

bool raycast_service::run_broadphase_async(job &completion_job) {
    if (m_broad_ctx.empty()) {
        m_state = state::done;
        return true;
    }

    m_state = state::broadphase;
    auto &bphase = m_registry->ctx().at<broadphase>();

    if (m_broad_ctx.size() <= m_max_raycast_broadphase_sequential_size) {
        run_broadphase();
        return true;
    }

    auto &dispatcher = job_dispatcher::global();
    auto *raycasts = &m_broad_ctx;

    parallel_for_async(dispatcher, size_t{}, raycasts->size(), size_t{1}, completion_job, [raycasts, &bphase](size_t index) {
        auto &ctx = (*raycasts)[index];
        bphase.raycast(ctx.p0, ctx.p1, [&](entt::entity entity) {
            if (!vector_contains(ctx.ignore_entities, entity)) {
                ctx.candidates.push_back(entity);
            }
        });
    });

    return false;
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

void raycast_service::run_narrowphase() {
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

bool raycast_service::run_narrowphase_async(job &completion_job) {
    if (m_narrow_ctx.empty()) {
        m_state = state::done;
        return true;
    }

    m_state = state::narrowphase;

    if (m_narrow_ctx.size() <= m_max_raycast_narrowphase_sequential_size) {
        run_narrowphase();
        return true;
    }

    auto index_view = m_registry->view<shape_index>();
    auto tr_view = m_registry->view<position, orientation>();
    auto origin_view = m_registry->view<origin>();
    auto shape_views_tuple = get_tuple_of_shape_views(*m_registry);
    auto &dispatcher = job_dispatcher::global();
    auto *ctxes = &m_narrow_ctx;

    parallel_for_async(dispatcher, size_t{}, ctxes->size(), size_t{1}, completion_job,
        [ctxes, index_view, origin_view, tr_view, shape_views_tuple](size_t index) {
        auto &ctx = (*ctxes)[index];

        auto sh_idx = index_view.get<shape_index>(ctx.entity);
        auto pos = origin_view.contains(ctx.entity) ?
            static_cast<vector3>(origin_view.get<origin>(ctx.entity)) : tr_view.get<position>(ctx.entity);
        auto orn = tr_view.get<orientation>(ctx.entity);
        auto ray_ctx = raycast_context{pos, orn, ctx.p0, ctx.p1};

        visit_shape(sh_idx, ctx.entity, shape_views_tuple, [&](auto &&shape) {
            ctx.result = shape_raycast(shape, ray_ctx);
        });
    });

    return false;
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

    m_state = state::done;
}

void raycast_service::update() {
    run_broadphase();
    finish_broadphase();
    run_narrowphase();
    finish_narrowphase();
}

}
