#ifndef __MUNIT_H__
#define __MUNIT_H__

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>


/* GLOBAL VARIABLES */
static int tests = 0;
static int passed = 0;
static int failed = 0;


/* MACROS */
#define KNRM  "\x1B[1;0m"
#define KRED  "\x1B[1;31m"
#define KGRN  "\x1B[1;32m"
#define KYEL  "\x1B[1;33m"
#define KBLU  "\x1B[1;34m"
#define KMAG  "\x1B[1;35m"
#define KCYN  "\x1B[1;36m"
#define KWHT  "\x1B[1;37m"

/* COMPARATOR FUNCTIONS */
#define EPSILON 0.0001
inline int fltcmp(float x, float y)
{
    if (fabs(x - y) < EPSILON) {
        return 0;
    } else if (x > y) {
        return 1;
    } else {
        return -1;
    }
}

/* MUNIT */
#define mu_assert(test, message) \
    do { \
        if (!(test)) { \
            printf( \
                "%sERROR!%s [%s:%d] %s\n", \
                KRED, \
                KNRM, \
                __func__, \
                __LINE__, \
                message \
            ); \
            return -1; \
        }\
    } while (0)

#define mu_check(test) \
    do { \
        if (!(test)) { \
            printf( \
                "%sERROR!%s [%s:%d] %s %sFAILED!%s\n", \
                KRED, \
                KNRM, \
                __func__, \
                __LINE__, \
                #test, \
                KRED, \
                KNRM \
            ); \
            return -1; \
        }\
    } while (0)

#define mu_add_test(test) \
    do { \
        tests++; \
        printf( \
            "%s-> %s %s\n", \
            KBLU, \
            #test, \
            KNRM \
        ); \
        fflush(stdout); \
        if (test() == -1) { \
            printf("%sTEST FAILED!%s\n\n", \
                KRED, \
                KNRM \
            ); \
            failed++; \
        } else { \
            printf("%sTEST PASSED!%s\n\n", \
                KGRN, \
                KNRM \
            ); \
            passed++; \
        } \
    } while (0)

#if defined(MU_PRINT)
  #if MU_PRINT == 1
    #define mu_print(message, ...) \
        printf(message, ##__VA_ARGS__)
  #elif MU_PRINT == 0
    #define mu_print(message, ...)
  #endif
#else
    #define mu_print(message, ...) \
        printf(message, ##__VA_ARGS__)
#endif

#define mu_report() \
    do { \
        printf(KBLU); \
        printf("%d tests, ", tests); \
        printf("%d passed, ", passed); \
        printf("%d failed\n", tests - passed); \
        printf("\n"); \
        printf(KNRM); \
        if (failed != 0) return -1; \
        else return 0; \
    } while (0)

#define mu_run_tests(TEST_SUITE) \
    int main(void) \
    { \
        TEST_SUITE(); \
        mu_report(); \
        return 0; \
    }

#endif
