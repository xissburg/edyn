#ifndef EDYN_CONFIG_EXECUTION_MODE_HPP
#define EDYN_CONFIG_EXECUTION_MODE_HPP

namespace edyn {

/**
 * @brief Mode of execution of physics simulation.
 */
enum class execution_mode {
    /**
     * Run physics simulation in main thread. The steps are performed
     * sequentially in every call to `edyn::update`.
     */
    sequential,

    /**
     * Run physics simulation partially and potentially in worker threads.
     * It blocks the main thread until all tasks are finished in the call
     * to `edyn::update`. Tasks are only parallelized if large enough.
     */
    sequential_multithreaded,

    /**
     * Run physics simulation in background threads. The call to `edyn::update`
     * does a minimal amount of work, mostly merging the simulation state from
     * workers in the main registry.
     */
    asynchronous
};

}

#endif // EDYN_CONFIG_EXECUTION_MODE_HPP
