#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <mutex>
#include <numeric>
#include <iterator>
#include "edyn/parallel/parallel_for_job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/config/config.h"

namespace edyn {

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const Function &func) {
    EDYN_ASSERT(step > IndexType{0});

    using job_type = parallel_for_job<IndexType, Function>;

    auto root_job = job_type(dispatcher, nullptr, first, last, step, &func);
    root_job.run();
    root_job.join();
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