#pragma once
#include <groot/groot.hpp>

#ifdef GROOT_GRAPH_DLL // defined if GROOT_GRAPH is compiled as a DLL
  #ifdef GROOT_GRAPH_DLL_EXPORTS // defined if we are building the GROOT_GRAPH DLL (instead of using it)
    #define GROOT_GRAPH_API GROOT_HELPER_DLL_EXPORT
  #else
    #define GROOT_GRAPH_API GROOT_HELPER_DLL_IMPORT
  #endif // GROOT_GRAPH_DLL_EXPORTS
  #define GROOT_GRAPH_LOCAL GROOT_HELPER_DLL_LOCAL
#else // GROOT_DLL is not defined: this means GROOT is a static lib.
  #define GROOT_GRAPH_API
  #define GROOT_GRAPH_LOCAL
#endif // GROOT_DLL