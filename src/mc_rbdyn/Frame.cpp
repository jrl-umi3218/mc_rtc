#include <mc_rbdyn/Frame.h>

namespace mc_rbdyn
{

Frame::Frame(NewFrameToken, const std::string & name) noexcept : name_(name) {}

Frame::Frame(NewFrameToken tkn,
             const std::string & name,
             Frame & parent,
             const sva::PTransformd & X_p_f,
             bool bake) noexcept
: Frame(tkn, name)
{
  if(bake)
  {
    position_ = X_p_f * parent.position();
    velocity_ = X_p_f * parent.velocity();
  }
  else
  {
    parent_ = parent;
    position_ = X_p_f;
  }
}

} // namespace mc_rbdyn
