/*
* Copyright 2023 LeoDrive.ai, Inc. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

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
