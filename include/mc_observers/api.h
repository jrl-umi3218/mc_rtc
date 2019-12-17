/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define MC_OBSERVERS_DLLIMPORT __declspec(dllimport)
#  define MC_OBSERVERS_DLLEXPORT __declspec(dllexport)
#  define MC_OBSERVERS_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MC_OBSERVERS_DLLIMPORT __attribute__((visibility("default")))
#    define MC_OBSERVERS_DLLEXPORT __attribute__((visibility("default")))
#    define MC_OBSERVERS_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MC_OBSERVERS_DLLIMPORT
#    define MC_OBSERVERS_DLLEXPORT
#    define MC_OBSERVERS_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MC_OBSERVERS_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MC_OBSERVERS_DLLAPI
#  define MC_OBSERVERS_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MC_OBSERVERS_EXPORTS
#    define MC_OBSERVERS_DLLAPI MC_OBSERVERS_DLLEXPORT
#  else
#    define MC_OBSERVERS_DLLAPI MC_OBSERVERS_DLLIMPORT
#  endif // MC_OBSERVERS_EXPORTS
#  define MC_OBSERVERS_LOCAL MC_OBSERVERS_DLLLOCAL
#endif // MC_OBSERVERS_STATIC

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define MC_OBSERVER_DLLIMPORT __declspec(dllimport)
#  define MC_OBSERVER_DLLEXPORT __declspec(dllexport)
#  define MC_OBSERVER_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MC_OBSERVER_DLLIMPORT __attribute__((visibility("default")))
#    define MC_OBSERVER_DLLEXPORT __attribute__((visibility("default")))
#    define MC_OBSERVER_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MC_OBSERVER_DLLIMPORT
#    define MC_OBSERVER_DLLEXPORT
#    define MC_OBSERVER_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MC_OBSERVER_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MC_OBSERVER_DLLAPI
#  define MC_OBSERVER_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MC_OBSERVER_EXPORTS
#    define MC_OBSERVER_DLLAPI MC_OBSERVER_DLLEXPORT
#  else
#    define MC_OBSERVER_DLLAPI MC_OBSERVER_DLLIMPORT
#  endif // MC_OBSERVER_EXPORTS
#  define MC_OBSERVER_LOCAL MC_OBSERVER_DLLLOCAL
#endif // MC_OBSERVER_STATIC
