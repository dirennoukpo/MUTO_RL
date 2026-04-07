#pragma once

#if defined(MUTO_LINK_STATIC)
  #define MUTO_LINK_API
#elif defined(_WIN32) || defined(__CYGWIN__)
  #if defined(MUTO_LINK_CPP_EXPORTS)
    #define MUTO_LINK_API __declspec(dllexport)
  #else
    #define MUTO_LINK_API __declspec(dllimport)
  #endif
#else
  #if defined(__GNUC__) || defined(__clang__)
    #define MUTO_LINK_API __attribute__((visibility("default")))
  #else
    #define MUTO_LINK_API
  #endif
#endif
