#include <mc_rtc/logging.h>

int main()
{
  mc_rtc::log::notify("This is a test notification with antislash \\ and \"quotes\"!\"\"");
  return 0;
}
