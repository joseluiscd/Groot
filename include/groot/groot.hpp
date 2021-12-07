#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define GROOT_HELPER_DLL_IMPORT __declspec(dllimport)
  #define GROOT_HELPER_DLL_EXPORT __declspec(dllexport)
  #define GROOT_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define GROOT_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define GROOT_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define GROOT_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define GROOT_HELPER_DLL_IMPORT
    #define GROOT_HELPER_DLL_EXPORT
    #define GROOT_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define GROOT_API and GROOT_LOCAL.
// GROOT_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// GROOT_LOCAL is used for non-api symbols.

#ifdef GROOT_DLL // defined if GROOT is compiled as a DLL
  #ifdef GROOT_DLL_EXPORTS // defined if we are building the GROOT DLL (instead of using it)
    #define GROOT_API GROOT_HELPER_DLL_EXPORT
  #else
    #define GROOT_API GROOT_HELPER_DLL_IMPORT
  #endif // GROOT_DLL_EXPORTS
  #define GROOT_LOCAL GROOT_HELPER_DLL_LOCAL
#else // GROOT_DLL is not defined: this means GROOT is a static lib.
  #define GROOT_API
  #define GROOT_LOCAL
#endif // GROOT_DLL