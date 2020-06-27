#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <iterator>
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/config/config.h"

namespace edyn {

template<typename IndexType, typename Function>
struct parallel_for_job: public job {
    IndexType m_first;
    IndexType m_last;
    IndexType m_step;
    const Function *m_func;

    parallel_for_job(IndexType first, IndexType last, IndexType step, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_func(func)
    {}

    void run() override {
        for (auto i = m_first; i < m_last; i += m_step) {
            (*m_func)(i);
        }
    }
};

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const Function &func) {
    EDYN_ASSERT(step > IndexType{0});
    auto count = (last - first) / step;
    auto num_workers = dispatcher.num_workers();
    auto items_per_worker = count / num_workers;
    std::vector<std::shared_ptr<job>> jobs;
    jobs.reserve(num_workers);

    for (size_t i = 0; i < num_workers; ++i) {
        auto i_first = IndexType{i} * step * items_per_worker;
        auto i_last = i == num_workers - 1 ? last : IndexType{i + 1} * step * items_per_worker;
        auto j = std::make_shared<parallel_for_job<IndexType, Function>>(i_first, i_last, step, &func);
        jobs.push_back(j);
        dispatcher.async(j);
    }

    for (auto &j : jobs) {
        j->join();
    }
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, IndexType step, const Function &func) {
    parallel_for(job_dispatcher::shared(), first, last, step, func);
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, const Function &func) {
    parallel_for(first, last, IndexType {1}, func);
}

template<typename Iterator, typename Function>
struct parallel_for_each_job: public job {
    Iterator m_first;
    Iterator m_last;
    const Function *m_func;

    parallel_for_each_job(Iterator first, Iterator last, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_func(func)
    {}

    void run() override {
        for (auto it = m_first; it != m_last; ++it) {
            (*m_func)(*it);
        }
    }
};

template<typename Iterator, typename Function>
void parallel_for_each(job_dispatcher &dispatcher, Iterator first, Iterator last, const Function &func) {
    auto count = std::distance(first, last);
    auto num_workers = dispatcher.num_workers();
    auto items_per_worker = count / num_workers;
    std::vector<std::shared_ptr<job>> jobs;
    jobs.reserve(num_workers);

    for (size_t i = 0; i < num_workers; ++i) {
        auto i_first = first + (i * items_per_worker);
        auto i_last = i == num_workers - 1 ? last : first + ((i + 1) * items_per_worker);
        auto j = std::make_shared<parallel_for_each_job>(i_first, i_last, &func);
        jobs.push_back(j);
        dispatcher.async(j);
    }

    for (auto &j : jobs) {
        j->join();
    }
}

template<typename Iterator, typename Function>
void parallel_for_each(Iterator first, Iterator last, const Function &func) {
    parallel_for_each(job_dispatcher::shared(), first, last, func);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_HPP