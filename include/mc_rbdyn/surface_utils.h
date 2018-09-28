#ifndef _H_MCRBDYNSURFACEUTILS_H_
#define _H_MCRBDYNSURFACEUTILS_H_

#include <mc_rbdyn/api.h>

#include <memory>
#include <string>
#include <vector>

namespace mc_rbdyn
{

struct Surface;

MC_RBDYN_DLLAPI std::vector<std::shared_ptr<Surface>> readRSDFFromDir(const std::string & dirname);

} // namespace mc_rbdyn

#endif
