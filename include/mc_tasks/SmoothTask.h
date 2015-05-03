#ifndef _H_SMOOTHTASK_H_
#define _H_SMOOTHTASK_H_

#include <functional>

template<typename objT>
struct SmoothTask
{
public:
  typedef std::function<void(double)> w_set_fn;
  typedef std::function<double(void)> w_get_fn;
  typedef std::function<void(const objT &)> obj_set_fn;
  typedef std::function<const objT(void)> obj_get_fn;
public:
  SmoothTask(w_set_fn w_set, w_get_fn w_get,
             obj_set_fn obj_set, obj_get_fn obj_get,
             double weight, const objT & obj, double percent);

  void update();

  void reset(double weight, const objT & obj, double percent);
public:
  w_set_fn w_set;
  w_get_fn w_get;
  obj_set_fn obj_set;
  obj_get_fn obj_get;

  double weight;
  objT obj;

  double stepW;
  objT stepO;

  unsigned int nrIter;
  unsigned int iter;
};

template<typename objT>
SmoothTask<objT>::SmoothTask(w_set_fn w_set, w_get_fn w_get,
                       obj_set_fn obj_set, obj_get_fn obj_get,
                       double weight, const objT & obj, double percent)
: w_set(w_set), w_get(w_get), obj_set(obj_set), obj_get(obj_get)
{
  reset(weight, obj, percent);
}

template<typename objT>
void SmoothTask<objT>::reset(double weight, const objT & obj, double percent)
{
  this->weight = weight;
  this->obj = obj;

  stepW = (weight - w_get())*percent;
  stepO = (obj - obj_get())*percent;

  nrIter = ceil(1/percent);
  iter = 0;
}

template<typename objT>
void SmoothTask<objT>::update()
{
  if(iter < nrIter)
  {
    w_set(w_get() + stepW);
    obj_set(obj_get() + stepO);
  }
  else
  {
    obj_set(obj);
    w_set(weight);
  }
}

#endif
