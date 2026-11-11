#ifndef UNITY_INTERNALS_H
#define UNITY_INTERNALS_H

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int NumberOfTests;
    int TestFailures;
    int TestIgnores;
    int CurrentTestFailed;
    const char* CurrentTestName;
    int CurrentTestLineNumber;
} UnityStorageType;

extern UnityStorageType Unity;

void UnityBegin(const char* filename);
int UnityEnd(void);
void UnityTestRunner(void (*func)(void), const char* name, int lineNumber);
void UnityAssertTrue(int condition, const char* msg, int lineNumber);
void UnityAssertEqualInt(int expected, int actual, int lineNumber);
void UnityAssertEqualUInt32(uint32_t expected, uint32_t actual, int lineNumber);
void UnityAssertFloatWithin(float delta, float expected, float actual, int lineNumber);
void UnityAssertEqualFloat(float expected, float actual, int lineNumber);
void UnityAssertLessOrEqual(double a, double b, int lineNumber);
void UnityAssertEqualString(const char* expected, const char* actual, int lineNumber);

#ifdef __cplusplus
}
#endif

#endif // UNITY_INTERNALS_H
