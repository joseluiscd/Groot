#pragma once

#include <cstdlib>
#include <cstdio>


#ifdef NDEBUG
#define GROOT_ASSERT(...)
#define GROOT_CHECK(...)
#else
#define GROOT_ASSERT(CONDITION, MSG, ...) if (!static_cast<bool>(CONDITION)) { fprintf(stderr, "%s:%d: Assertion failed: %s\n", __FILE__, __LINE__, #CONDITION); fprintf(stderr, MSG "\n", ##__VA_ARGS__); abort(); }
#define GROOT_CHECK(CONDITION, MSG, ...) if (!static_cast<bool>(CONDITION)) { fprintf(stderr, "%s:%d: Assertion failed: %s\n", __FILE__, __LINE__, #CONDITION); fprintf(stderr, MSG "\n", ##__VA_ARGS__); }
#endif