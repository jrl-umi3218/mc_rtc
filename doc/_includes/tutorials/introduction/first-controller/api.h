#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MyFirstController_DLLIMPORT __declspec(dllimport)
#  define MyFirstController_DLLEXPORT __declspec(dllexport)
#  define MyFirstController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MyFirstController_DLLIMPORT __attribute__((visibility("default")))
#    define MyFirstController_DLLEXPORT __attribute__((visibility("default")))
#    define MyFirstController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MyFirstController_DLLIMPORT
#    define MyFirstController_DLLEXPORT
#    define MyFirstController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MyFirstController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MyFirstController_DLLAPI
#  define MyFirstController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MyFirstController_EXPORTS
#    define MyFirstController_DLLAPI MyFirstController_DLLEXPORT
#  else
#    define MyFirstController_DLLAPI MyFirstController_DLLIMPORT
#  endif // MyFirstController_EXPORTS
#  define MyFirstController_LOCAL MyFirstController_DLLLOCAL
#endif // MyFirstController_STATIC
