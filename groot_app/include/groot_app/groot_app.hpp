#pragma once
#include <groot/groot.hpp>

#ifdef GROOT_APP_DLL // defined if GROOT_APP is compiled as a DLL
  #ifdef GROOT_APP_DLL_EXPORTS // defined if we are building the GROOT_GRAPH DLL (instead of using it)
    #define GROOT_APP_API GROOT_HELPER_DLL_EXPORT
  #else
    #define GROOT_APP_API GROOT_HELPER_DLL_IMPORT
  #endif // GROOT_GRAPH_DLL_EXPORTS
  #define GROOT_APP_LOCAL GROOT_HELPER_DLL_LOCAL
#else // GROOT_DLL is not defined: this means GROOT is a static lib.
  #define GROOT_APP_API
  #define GROOT_APP_LOCAL
#endif // GROOT_DLL