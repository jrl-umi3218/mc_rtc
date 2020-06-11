#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <iostream>

void usage(const char * prog)
{
  std::cerr << prog << " [json] ([yaml])\n";
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage(argv[0]);
    return 1;
  }
  bfs::path in(argv[1]);
  bfs::path out;
  if(argc > 2)
  {
    out = bfs::path(argv[2]);
  }
  else
  {
    out = in;
    out.replace_extension(".yaml");
    mc_rtc::log::info("Output conversion to {}", out);
  }
  mc_rtc::Configuration conf(in.string());
  conf.save(out.string());
  return 0;
}
