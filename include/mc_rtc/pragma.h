/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** Preprocessor identity, used to force expansion (mainly to work around MSVC "feature" */
#define MC_RTC_PP_ID(x) x

/** Transform a expression to string*/
#define MC_RTC_STRINGIFY_(x) #x
#define MC_RTC_STRINGIFY(x) MC_RTC_STRINGIFY_(x)

#define MC_RTC_PRAGMA(x) _Pragma(MC_RTC_STRINGIFY(x))

/** Signal wrong number of arguments */
#define MC_RTC_WN(fun, ...)                                                                                          \
  MC_RTC_PRAGMA(message(__FILE__ "[" STRING(__LINE__) "] Incorrect number of arguments when invoking macro " STRING( \
      fun) ". Call is ignored"))

/** Sub-function in FOR_EACH macro reading arguments two by two */
#define MC_RTC_MAP2_0(fun)
#define MC_RTC_MAP2_1(fun, X, Y) fun(X, Y)
#define MC_RTC_MAP2_2(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_1(fun, __VA_ARGS__))
#define MC_RTC_MAP2_3(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_2(fun, __VA_ARGS__))
#define MC_RTC_MAP2_4(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_3(fun, __VA_ARGS__))
#define MC_RTC_MAP2_5(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_4(fun, __VA_ARGS__))
#define MC_RTC_MAP2_6(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_5(fun, __VA_ARGS__))
#define MC_RTC_MAP2_7(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_6(fun, __VA_ARGS__))
#define MC_RTC_MAP2_8(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_7(fun, __VA_ARGS__))
#define MC_RTC_MAP2_9(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_8(fun, __VA_ARGS__))
#define MC_RTC_MAP2_10(fun, X, Y, ...) fun(X, Y) MC_RTC_PP_ID(MC_RTC_MAP2_9(fun, __VA_ARGS__))

#define MC_RTC_GET_MACRO(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, \
                         _20, NAME, ...)                                                                           \
  NAME

/** Transform fun(arg1, arg2, arg3, arg4, ...) into fun(arg1, arg2), fun(arg3, arg4), ...*/
#define MC_RTC_MAP_TWO_ARGS(fun, ...)                                                                                \
  MC_RTC_PP_ID(MC_RTC_GET_MACRO(_0, __VA_ARGS__, MC_RTC_MAP2_10, MC_RTC_WN, MC_RTC_MAP2_9, MC_RTC_WN, MC_RTC_MAP2_8, \
                                MC_RTC_WN, MC_RTC_MAP2_7, MC_RTC_WN, MC_RTC_MAP2_6, MC_RTC_WN, MC_RTC_MAP2_5,        \
                                MC_RTC_WN, MC_RTC_MAP2_4, MC_RTC_WN, MC_RTC_MAP2_3, MC_RTC_WN, MC_RTC_MAP2_2,        \
                                MC_RTC_WN, MC_RTC_MAP2_1, MC_RTC_WN, MC_RTC_MAP2_0, MC_RTC_MAP2_0)(fun, __VA_ARGS__))

#ifdef __GNUG__
#  define MC_RTC_diagnostic_push _Pragma("GCC diagnostic push")
#  define MC_RTC_diagnostic_pop _Pragma("GCC diagnostic pop")
#  define MC_RTC_GCC_diagnostic_ignored(x) MC_RTC_PRAGMA(GCC diagnostic ignored x)
#  ifdef __clang__
#    define MC_RTC_ClangOnly_diagnostic_ignored(x) MC_RTC_PRAGMA(GCC diagnostic ignored x)
#    define MC_RTC_GCCOnly_diagnostic_ignored(x)
#  else
#    define MC_RTC_ClangOnly_diagnostic_ignored(x)
#    define MC_RTC_GCCOnly_diagnostic_ignored(x) MC_RTC_PRAGMA(GCC diagnostic ignored x)
#  endif
#else
#  define MC_RTC_GCC_diagnostic_ignored(x)
#  define MC_RTC_ClangOnly_diagnostic_ignored(x)
#  define MC_RTC_GCCOnly_diagnostic_ignored(x)
#endif

#ifdef _MSC_VER
#  define MC_RTC_diagnostic_push _Pragma("warning(push)")
#  define MC_RTC_diagnostic_pop _Pragma("warning(pop)")
#  define MC_RTC_MSVC_diagnostic_ignored(x) MC_RTC_PRAGMA(warning(disable : x))
#else
#  define MC_RTC_MSVC_diagnostic_ignored(x)
#endif

/** Helper macro for MC_RTC_diagnostic_ignored*/
#define MC_RTC_diagnostic_ignored_(c, w) MC_RTC_##c##_diagnostic_ignored(w)

/** Ignore warning by compiler. Used as MC_RTC_diagnostic_ignored(compiler1, warningID1, compiler2, warningID2, ...).*/
#define MC_RTC_diagnostic_ignored(...) MC_RTC_MAP_TWO_ARGS(MC_RTC_diagnostic_ignored_, __VA_ARGS__)