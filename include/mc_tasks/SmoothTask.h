#pragma once

#include <functional>

namespace mc_tasks
{

/*! \brief SmoothTask allows to smoothly reach a final weight and objective for
 * a given task
 *
 * \tparam objT The type of the objective given to the task
 */
template<typename objT>
struct SmoothTask
{
public:
  /*! \brief A function to set the weight of the task */
  typedef std::function<void(double)> w_set_fn;
  /*! \brief A function to get the current weight of the task */
  typedef std::function<double(void)> w_get_fn;
  /*! \brief A function to set the objective of a task */
  typedef std::function<void(const objT &)> obj_set_fn;
  /*! \brief A function to get the objective of a task */
  typedef std::function<const objT(void)> obj_get_fn;

public:
  /*! \brief Constructor
   *
   * \param w_set Weight setting function
   *
   * \param w_get Weight getting function
   *
   * \param obj_set Objective setting function
   *
   * \param obj_get Objective getting function
   *
   * \param weight The target weight
   *
   * \param obj The target objective
   *
   * \param percent The interpolation speed as a percentage (step-size, 1 is
   * instantaneous, 0 stop the smooth task process)
   *
   */
  SmoothTask(w_set_fn w_set,
             w_get_fn w_get,
             obj_set_fn obj_set,
             obj_get_fn obj_get,
             double weight,
             const objT & obj,
             double percent);

  /*! \brief Update the task
   *
   * This will move the weight to the target weight and the objective to the
   * target objective
   *
   */
  void update();

  /*! \brief Reset the target weight, target objective and interpolation step
   *
   * \param weight New target weight
   *
   * \param obj New objective
   *
   * \param percent The interpolation speed as a percentage (step-size, 1 is
   * instantaneous, 0 stop the smooth task process)
   *
   */
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
SmoothTask<objT>::SmoothTask(w_set_fn w_set,
                             w_get_fn w_get,
                             obj_set_fn obj_set,
                             obj_get_fn obj_get,
                             double weight,
                             const objT & obj,
                             double percent)
: w_set(w_set), w_get(w_get), obj_set(obj_set), obj_get(obj_get)
{
  reset(weight, obj, percent);
}

template<typename objT>
void SmoothTask<objT>::reset(double weight, const objT & obj, double percent)
{
  this->weight = weight;
  this->obj = obj;

  stepW = (weight - w_get()) * percent;
  stepO = (obj - obj_get()) * percent;

  nrIter = static_cast<unsigned int>(ceil(1 / percent));
  iter = 0;
}

template<typename objT>
void SmoothTask<objT>::update()
{
  if(iter < nrIter)
  {
    w_set(w_get() + stepW);
    obj_set(obj_get() + stepO);
    iter++;
  }
  else
  {
    obj_set(obj);
    w_set(weight);
  }
}

} // namespace mc_tasks
