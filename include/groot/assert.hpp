#pragma once

#include <cstdio>
#include <cstdlib>

#ifdef NDEBUG
#define GROOT_ASSERT(...)
#define GROOT_CHECK(...)
#else

#define GROOT_ASSERT(CONDITION, ...)                                                      \
    if (!static_cast<bool>(CONDITION)) {                                                  \
        fprintf(stderr, "%s:%d: Assertion failed: %s\n", __FILE__, __LINE__, #CONDITION); \
        fprintf(stderr, __VA_ARGS__);                                                     \
        fputs("\n", stderr);                                                              \
        abort();                                                                          \
    }
#define GROOT_CHECK(CONDITION, ...)                                                   \
    if (!static_cast<bool>(CONDITION)) {                                              \
        fprintf(stderr, "%s:%d: Check failed: %s\n", __FILE__, __LINE__, #CONDITION); \
        fprintf(stderr, __VA_ARGS__);                                                 \
        fputs("\n", stderr);                                                          \
    }
#endif