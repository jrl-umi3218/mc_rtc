#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CoMTrajectoryGeneration_DLLIMPORT __declspec(dllimport)
#  define CoMTrajectoryGeneration_DLLEXPORT __declspec(dllexport)
#  define CoMTrajectoryGeneration_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CoMTrajectoryGeneration_DLLIMPORT __attribute__((visibility("default")))
#    define CoMTrajectoryGeneration_DLLEXPORT __attribute__((visibility("default")))
#    define CoMTrajectoryGeneration_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CoMTrajectoryGeneration_DLLIMPORT
#    define CoMTrajectoryGeneration_DLLEXPORT
#    define CoMTrajectoryGeneration_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CoMTrajectoryGeneration_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CoMTrajectoryGeneration_DLLAPI
#  define CoMTrajectoryGeneration_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CoMTrajectoryGeneration_EXPORTS
#    define CoMTrajectoryGeneration_DLLAPI CoMTrajectoryGeneration_DLLEXPORT
#  else
#    define CoMTrajectoryGeneration_DLLAPI CoMTrajectoryGeneration_DLLIMPORT
#  endif // CoMTrajectoryGeneration_EXPORTS
#  define CoMTrajectoryGeneration_LOCAL CoMTrajectoryGeneration_DLLLOCAL
#endif // CoMTrajectoryGeneration_STATIC