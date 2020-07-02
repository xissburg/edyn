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

    parallel_for(dispatcher, size_t{0}, size_t{count}, size_t{1}, [&] (size_t index) {
        func(first + index);
    });
}

template<typename Iterator, typename Function>
void parallel_for_each(Iterator first, Iterator last, const Function &func) {
    parallel_for_each(job_dispatcher::shared(), first, last, func);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_HPP