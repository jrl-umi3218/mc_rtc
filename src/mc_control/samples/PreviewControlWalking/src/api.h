#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define PreviewControlWalking_DLLIMPORT __declspec(dllimport)
#  define PreviewControlWalking_DLLEXPORT __declspec(dllexport)
#  define PreviewControlWalking_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define PreviewControlWalking_DLLIMPORT __attribute__((visibility("default")))
#    define PreviewControlWalking_DLLEXPORT __attribute__((visibility("default")))
#    define PreviewControlWalking_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define PreviewControlWalking_DLLIMPORT
#    define PreviewControlWalking_DLLEXPORT
#    define PreviewControlWalking_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef PreviewControlWalking_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PreviewControlWalking_DLLAPI
#  define PreviewControlWalking_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PreviewControlWalking_EXPORTS
#    define PreviewControlWalking_DLLAPI PreviewControlWalking_DLLEXPORT
#  else
#    define PreviewControlWalking_DLLAPI PreviewControlWalking_DLLIMPORT
#  endif // PreviewControlWalking_EXPORTS
#  define PreviewControlWalking_LOCAL PreviewControlWalking_DLLLOCAL
#endif // PreviewControlWalking_STATIC