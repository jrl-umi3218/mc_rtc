#ifndef _H_MCRBDYNSURFACEUTILS_H_
#define _H_MCRBDYNSURFACEUTILS_H_

#include <memory>
#include <string>
#include <vector>

namespace mc_rbdyn
{

struct Surface;

std::vector< std::shared_ptr<Surface> > readRSDFFromDir(const std::string & dirname);

}

#endif
