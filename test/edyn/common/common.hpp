#ifndef TEST_EDYN_COMMON_COMMON_HPP
#define TEST_EDYN_COMMON_COMMON_HPP

#include <gtest/gtest.h>
#include <edyn/build_settings.h>

#if EDYN_DOUBLE_PRECISION
#define ASSERT_SCALAR_EQ ASSERT_DOUBLE_EQ
#else
#define ASSERT_SCALAR_EQ ASSERT_FLOAT_EQ
#endif

#endif // TEST_EDYN_COMMON_COMMON_HPP