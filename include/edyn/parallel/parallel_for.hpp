#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <mutex>
#include <numeric>
#include <iterator>
#include <future>
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/config/config.h"

namespace edyn {

template<typename IndexType, typename Function>
class parallel_for_job: public job {
public:
    parallel_for_job(IndexType first, IndexType last, 
                     IndexType step, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_func(func)
    {}

    void run() override {
        for (auto i = m_first; i < m_last; i += m_step) {
            (*m_func)(i);
        }
        m_promise.set_value();
    }

    void join() {
        m_promise.get_future().get();
    }

private:
    IndexType m_first;
    IndexType m_last;
    IndexType m_step;
    const Function *m_func;
    std::promise<void> m_promise;
};

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const Function &func) {
    EDYN_ASSERT(step > IndexType{0});
    auto count = (last - first) / step;
    // Create N background jobs and run the last job in the calling thread.
    auto num_jobs = dispatcher.num_workers() + 1;
    auto items_per_job = count / num_jobs;

    using job_type = parallel_for_job<IndexType, Function>;
    std::vector<std::shared_ptr<job_type>> jobs;
    jobs.reserve(num_jobs - 1);

    for (size_t i = 0; i < num_jobs - 1; ++i) {
        auto i_first = IndexType{i} * step * items_per_job;
        auto i_last = IndexType{i + 1} * step * items_per_job;
        auto j = std::make_shared<job_type>(i_first, i_last, step, &func);
        jobs.push_back(j);
        dispatcher.async(j);
    }

    auto this_first = IndexType{num_jobs - 1} * step * items_per_job;
    auto this_last = last;
    for (auto i = this_first; i < this_last; i += step) {
        func(i);
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
class parallel_for_each_job: public job {
public:
    parallel_for_each_job(Iterator first, Iterator last, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_func(func)
    {}

    void run() override {
        for (auto it = m_first; it != m_last; ++it) {
            (*m_func)(*it);
        }
        m_promise.set_value();
    }

    void join() {
        m_promise.get_future().get();
    }

private:
    Iterator m_first;
    Iterator m_last;
    const Function *m_func;
    std::promise<void> m_promise;
};

template<typename Iterator, typename Function>
void parallel_for_each(job_dispatcher &dispatcher, Iterator first, Iterator last, const Function &func) {
    auto count = std::distance(first, last);
    // Create N background jobs and run the last job in the calling thread.
    auto num_jobs = dispatcher.num_workers() + 1;
    auto items_per_job = count / num_jobs;

    using job_type = parallel_for_each_job<Iterator, Function>;
    std::vector<std::shared_ptr<job_type>> jobs;
    jobs.reserve(num_jobs - 1);

    for (size_t i = 0; i < num_jobs - 1; ++i) {
        auto i_first = first + (i * items_per_job);
        auto i_last = first + ((i + 1) * items_per_job);
        auto j = std::make_shared<job_type>(i_first, i_last, &func);
        jobs.push_back(j);
        dispatcher.async(j);
    }

    auto this_first = first + ((num_jobs - 1) * items_per_job);
    auto this_last = last;
    for (auto it = this_first; it != this_last; ++it) {
        func(*it);
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