#include "edyn/time/time.hpp"

namespace edyn {

double performance_time()
{
    return static_cast<double>(performance_counter()) /
           static_cast<double>(performance_frequency());
}

}
