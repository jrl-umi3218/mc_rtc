#pragma once

#include <mc_rtc/logging.h>

#include <functional>

#ifdef __linux__
#include <unistd.h>
#include <sched.h>
#include <setjmp.h>
#include <signal.h>

namespace
{
  jmp_buf jmp;

  void signal_handler(int signal)
  {
    if(signal == SIGSEGV || signal == SIGFPE)
    {
      /*! Avoid jumping back a second time */
      static bool dead = false;
      if(!dead)
      {
        dead = true;
        longjmp(jmp, signal);
      }
    }
    _exit(0);
  }
}

#endif

namespace mc_rtc
{

/*! \brief Holds sandbox data
 *
 * This structure will be used to call fn in a light process and its
 * return is monitored by the parent process. This allows to recover
 * from any issue raised by the creation call
 *
 * \tparam T Object type created
 *
 */
template<typename T>
struct LoaderSandboxData
{
  std::function<T*(void)> fn;
  T * ret = nullptr;
  bool complete = false;
};

#ifdef __linux__
template<typename T>
int sandbox(void * args)
{
  LoaderSandboxData<T> & data = *(static_cast<LoaderSandboxData<T>*>(args));
  try
  {
    signal(SIGSEGV, signal_handler);
    signal(SIGFPE, signal_handler);
    int jmp_res = setjmp(jmp);
    if(jmp_res == 0)
    {
      data.ret = data.fn();
      data.complete = true;
    }
    else
    {
      data.ret = nullptr;
      data.complete = false;
      if(jmp_res == SIGSEGV)
      {
        LOG_ERROR("Loaded constructor segfaulted")
      }
      else
      {
        LOG_ERROR("Loaded constructor raised a floating-point exception")
      }
    }
    signal(SIGSEGV, SIG_DFL);
    signal(SIGFPE, SIG_DFL);
  }
  catch(...)
  {
    LOG_ERROR("Loaded constructor threw an exception")
    data.ret = nullptr;
  }
  _exit(0);
}
#endif

/*! \brief Calls a function in a sandbox
 *
 * Allows a function call to fail horribly without crashing the main
 * program, allowing us to survive the event, report and throw.
 *
 * Only works on Linux.
 *
 * \tparam T Return type of the function
 *
 * \tparam Args Arguments passed to the creation function
 *
 */
template<typename T, typename ... Args>
T * sandbox_function_call(std::function<T*(const Args & ...)> create_fn, const Args & ... args)
{
#ifdef __linux__
  LoaderSandboxData<T> data;
  data.fn = std::bind(create_fn, args...);
  unsigned int stack_size = 65536*100;
  char * stack = static_cast<char*>(malloc(stack_size*sizeof(char)));
  clone(sandbox<T>, static_cast<void*>(stack + stack_size),
        CLONE_FILES|CLONE_FS|CLONE_IO|CLONE_VM|CLONE_VFORK,
        static_cast<void*>(&data));
  if(data.complete)
  {
    return data.ret;
  }
  else
  {
    return nullptr;
  }
#else
  /* TODO Port to MacOS/WIN32 (good luck) */
  return no_sandbox_function_call(create_fn, args...);
#endif
}

/*! \brief Calls a function without sandboxing
 *
 * Only catches exceptions
 *
 * \tparam T Return type of the function
 *
 * \tparam Args Arguments passed to the creation function
 *
 */
template<typename T, typename ... Args>
T * no_sandbox_function_call(std::function<T*(const Args & ...)> create_fn, const Args & ... args)
{
  try
  {
    return create_fn(args...);
  }
  catch(std::exception & exc)
  {
    LOG_ERROR("Loaded constructor threw an exception")
    LOG_WARNING(exc.what())
  }
  catch(...)
  {
    LOG_ERROR("Loaded constructor threw an exception")
  }
  return nullptr;
}

} // namespace mc_rtc
