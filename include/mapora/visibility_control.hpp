// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__VISIBILITY_CONTROL_HPP_
#define MAPORA__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(MAPORA_BUILDING_DLL) || defined(MAPORA_EXPORTS)
    #define MAPORA_PUBLIC __declspec(dllexport)
    #define MAPORA_LOCAL
  #else  // defined(MAPORA_BUILDING_DLL) || defined(MAPORA_EXPORTS)
    #define MAPORA_PUBLIC __declspec(dllimport)
    #define MAPORA_LOCAL
  #endif  // defined(MAPORA_BUILDING_DLL) || defined(MAPORA_EXPORTS)
#elif defined(__linux__)
  #define MAPORA_PUBLIC __attribute__((visibility("default")))
  #define MAPORA_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MAPORA_PUBLIC __attribute__((visibility("default")))
  #define MAPORA_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // MAPORA__VISIBILITY_CONTROL_HPP_
