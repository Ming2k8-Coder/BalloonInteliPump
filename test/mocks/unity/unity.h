#ifndef UNITY_FRAMEWORK_H
#define UNITY_FRAMEWORK_H

#include "unity_internals.h"

#define UNITY_BEGIN() UnityBegin(__FILE__)
#define UNITY_END() UnityEnd()
#define RUN_TEST(func) UnityTestRunner(func, #func, __LINE__)

#define TEST_ASSERT_TRUE(c) UnityAssertTrue((c) ? 1 : 0, "Expected TRUE", __LINE__)
#define TEST_ASSERT_FALSE(c) UnityAssertTrue((c) ? 0 : 1, "Expected FALSE", __LINE__)
#define TEST_ASSERT_EQUAL_INT(e, a) UnityAssertEqualInt((int)(e), (int)(a), __LINE__)
#define TEST_ASSERT_EQUAL_UINT32(e, a) UnityAssertEqualUInt32((uint32_t)(e), (uint32_t)(a), __LINE__)
#define TEST_ASSERT_FLOAT_WITHIN(d, e, a) UnityAssertFloatWithin((float)(d), (float)(e), (float)(a), __LINE__)
#define TEST_ASSERT_EQUAL_FLOAT(e, a) UnityAssertEqualFloat((float)(e), (float)(a), __LINE__)
#define TEST_ASSERT_LESS_OR_EQUAL(a, b) UnityAssertLessOrEqual((double)(a), (double)(b), __LINE__)
#define TEST_ASSERT_EQUAL_STRING(e, a) UnityAssertEqualString((const char*)(e), (const char*)(a), __LINE__)

#endif // UNITY_FRAMEWORK_H
