#ifndef _H_CONTACTTRANSFORM_H_
#define _H_CONTACTTRANSFORM_H_

#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rbdyn/api.h>

namespace mc_rbdyn
{

struct Surface;

MC_RBDYN_DLLAPI sva::PTransformd planar(const double & T, const double & B, const double & N_rot);

MC_RBDYN_DLLAPI sva::PTransformd cylindrical(const double & T, const double & T_rot);

MC_RBDYN_DLLAPI void planarParam(const sva::PTransformd & X_es_rs, double & T, double & B, double & N_rot);

MC_RBDYN_DLLAPI void cylindricalParam(const sva::PTransformd & X_es_rs, double & T, double & T_rot);

MC_RBDYN_DLLAPI std::vector<double> jointParam(const Surface & r1Surface, const Surface & r2Surface, const sva::PTransformd & X_es_rs);

}

#endif
