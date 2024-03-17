#ifndef EDYN_CONFIG_CONFIG_H
#define EDYN_CONFIG_CONFIG_H

#ifndef EDYN_DISABLE_ASSERT
#include <cassert>
#define EDYN_ASSERT(condition, ...) assert(condition)
#else // EDYN_DISABLE_ASSERT
#undef EDYN_ASSERT
#define EDYN_ASSERT(...) ((void)0)
#endif // EDYN_DISABLE_ASSERT

#endif // EDYN_CONFIG_CONFIG_H
