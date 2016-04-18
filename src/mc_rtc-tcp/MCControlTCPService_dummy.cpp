#include "MCControlTCPService.h"

struct MCControlTCPServiceImpl
{
};

MCControlTCPService::MCControlTCPService(mc_control::MCGlobalController &) :
  impl(new MCControlTCPServiceImpl())
{
}

MCControlTCPService::~MCControlTCPService()
{
}
