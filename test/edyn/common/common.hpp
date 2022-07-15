#ifndef TEST_EDYN_COMMON_COMMON_HPP
#define TEST_EDYN_COMMON_COMMON_HPP

#include <gtest/gtest.h>
#include <edyn/edyn.hpp>

#ifdef EDYN_DOUBLE_PRECISION
#define ASSERT_SCALAR_EQ ASSERT_DOUBLE_EQ
#else
#define ASSERT_SCALAR_EQ ASSERT_FLOAT_EQ
#endif

inline void ASSERT_VECTOR3_EQ(edyn::vector3 v0, edyn::vector3 v1) {
    ASSERT_SCALAR_EQ(v0.x, v1.x);
    ASSERT_SCALAR_EQ(v0.y, v1.y);
    ASSERT_SCALAR_EQ(v0.z, v1.z);
}

#endif // TEST_EDYN_COMMON_COMMON_HPP
