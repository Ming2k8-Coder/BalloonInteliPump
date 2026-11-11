#include "unity.h"

UnityStorageType Unity = {0, 0, 0, 0, NULL, 0};

void UnityBegin(const char* filename) {
    (void)filename;
    Unity.NumberOfTests = 0;
    Unity.TestFailures = 0;
    Unity.TestIgnores = 0;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestLineNumber = 0;
}

int UnityEnd(void) {
    printf("-----------------------\n");
    printf("%d Tests %d Failures %d Ignored\n", Unity.NumberOfTests, Unity.TestFailures, Unity.TestIgnores);
    if (Unity.TestFailures == 0) {
        printf("OK\n");
        return 0;
    } else {
        printf("FAIL\n");
        return Unity.TestFailures;
    }
}

void UnityTestRunner(void (*func)(void), const char* name, int lineNumber) {
    Unity.NumberOfTests++;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestName = name;
    Unity.CurrentTestLineNumber = lineNumber;

    func();

    if (!Unity.CurrentTestFailed) {
        printf("%s:%d: %s: PASS\n", __FILE__, lineNumber, name);
    }
}

void UnityAssertTrue(int condition, const char* msg, int lineNumber) {
    if (!condition) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: %s\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", msg);
    }
}

void UnityAssertEqualInt(int expected, int actual, int lineNumber) {
    if (expected != actual) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: Expected %d Was %d\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", expected, actual);
    }
}

void UnityAssertEqualUInt32(uint32_t expected, uint32_t actual, int lineNumber) {
    if (expected != actual) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: Expected %u Was %u\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", (unsigned)expected, (unsigned)actual);
    }
}

void UnityAssertFloatWithin(float delta, float expected, float actual, int lineNumber) {
    if (fabsf(expected - actual) > delta) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: Expected %.4f Was %.4f (delta %.4f)\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", expected, actual, delta);
    }
}

void UnityAssertEqualFloat(float expected, float actual, int lineNumber) {
    UnityAssertFloatWithin(1e-5f, expected, actual, lineNumber);
}

void UnityAssertLessOrEqual(double a, double b, int lineNumber) {
    if (a > b) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: Expected %.4f <= %.4f\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", a, b);
    }
}

void UnityAssertEqualString(const char* expected, const char* actual, int lineNumber) {
    if (!expected || !actual || strcmp(expected, actual) != 0) {
        Unity.CurrentTestFailed = 1;
        Unity.TestFailures++;
        printf("%s:%d: %s: FAIL: Expected '%s' Was '%s'\n", __FILE__, lineNumber, Unity.CurrentTestName ? Unity.CurrentTestName : "Test", expected ? expected : "null", actual ? actual : "null");
    }
}
