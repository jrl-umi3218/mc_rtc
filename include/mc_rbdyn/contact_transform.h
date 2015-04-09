#ifndef _H_CONTACTTRANSFORM_H_
#define _H_CONTACTTRANSFORM_H_

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

struct Surface;

sva::PTransformd planar(const double & T, const double & B, const double & N_rot);

sva::PTransformd cylindrical(const double & T, const double & T_rot);

void planarParam(const sva::PTransformd & X_es_rs, double & T, double & B, double & N_rot);

void cylindricalParam(const sva::PTransformd & X_es_rs, double & T, double & T_rot);

}

#endif
