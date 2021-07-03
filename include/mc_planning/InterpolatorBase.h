#pragma once
#include <deque>

namespace mc_planning
{
namespace motion_interpolator
{

typedef enum
{
  NOT_SELECTED,
  // POINT_TO_POINT,
  // POINT_TO_POINT_EXP,
  // POINT_TO_POINT_CUBIC,
  // POINT_TO_POINT_TRAPEZOID_VELOCITY_PROFILES,
  // HOFFARBIB_MODEL,
  // NATURAL_CUBIC_SPLINE,
  CLAMPED_CUBIC_SPLINE,
  // AKIMA_CUBIC_SPLINE,
  // KOCHANEK_BARTELS_SPLINE,
  // BSPLINE_OPEN_UNIFORM_KNOT,
  // BSPLINE_OPEN_UNIFORM_KNOT_2ND,
  // BSPLINE_OPEN_UNIFORM_KNOT_3RD,
  // SINGLE_VIA_POINT,
  // DOUBLE_VIA_POINTS,
  // SEQUENTIAL_CUBIC_SPLINE,
  // SEQUENTIAL_QUINTIC_SPLINE,
  // TRAPEZOID_VELOCITY_PROFILES,
  // STOP_EXP,
  // STOP_QUINTIC_POLYNOMIALS,
  // MONOTONIC_INCREASING_EXP
} interpolator_type;

template<typename T>
class InterpolatorDataType
{
public:
  bool boundary;
  T t;
  double pos;
  double vel;
  double acc;

  InterpolatorDataType<T>(T t_ = 0.0, double pos_ = 0.0, double vel_ = 0.0, double acc_ = 0.0, bool boundary_ = false)
  : boundary(boundary_), t(t_), pos(pos_), vel(vel_), acc(acc_)
  {
  }

  InterpolatorDataType<T>(const InterpolatorDataType<T> & obj)
  : boundary(obj.boundary), t(obj.t), pos(obj.pos), vel(obj.vel), acc(obj.acc)
  {
  }
};

template<typename T>
class InterpolatorBase
{
private:
  interpolator_type ip_type;

protected:
  const double scale;
  const double eps;

  std::deque<InterpolatorDataType<T>> data;

public:
  InterpolatorBase(const double & scale_ = 1.0, const double & eps = 0.001, interpolator_type ip_type = NOT_SELECTED);

  virtual ~InterpolatorBase() {}

  inline const interpolator_type & InterpolatorType(void) const
  {
    return ip_type;
  }

  inline void clear(void)
  {
    data.clear();
  }

  inline unsigned int size(void)
  {
    return data.size();
  }

  inline void resize(unsigned int n)
  {
    data.resize(n);
  }

  inline bool empty(void)
  {
    return data.empty();
  }

  inline std::deque<InterpolatorDataType<T>> & getData(void)
  {
    return data;
  }

  inline InterpolatorDataType<T> & getData(int n)
  {
    return data[n];
  }

  inline InterpolatorDataType<T> & front(void)
  {
    return data.front();
  }

  inline InterpolatorDataType<T> & back(void)
  {
    return data.back();
  }

  inline virtual bool isFinish(const T & t_)
  {
    return (data.empty() || (data.back().t <= t_)) ? true : false;
  }

  inline void pop_front(void)
  {
    if(!data.empty()) data.pop_front();
  }

  inline void pop_back(void)
  {
    if(!data.empty()) data.pop_back();
  }

  inline void push_back(const T & t_, const double & p)
  {
    data.push_back(InterpolatorDataType<T>(t_, p));
  }

  inline void push_back(const T & t_, const double & p, const double & v)
  {
    data.push_back(InterpolatorDataType<T>(t_, p, v));
  }

  inline void push_back(const T & t_, const double & p, const double & v, const double & a)
  {
    data.push_back(InterpolatorDataType<T>(t_, p, v, a));
  }

  inline void push_back(const T & t_, const double & p, bool boundary)
  {
    data.push_back(InterpolatorDataType<T>(t_, p, 0.0, 0.0, boundary));
  }

  inline void push_back(const T & t_, const double & p, const double & v, bool boundary)
  {
    data.push_back(InterpolatorDataType<T>(t_, p, v, 0.0, boundary));
  }

  inline void push_back(const T & t_, const double & p, const double & v, const double & a, bool boundary)
  {
    data.push_back(InterpolatorDataType<T>(t_, p, v, a, boundary));
  }

  inline void push_front(const T & t_, const double & p)
  {
    data.push_front(InterpolatorDataType<T>(t_, p));
  }

  inline void push_front(const T & t_, const double & p, const double & v)
  {
    data.push_front(InterpolatorDataType<T>(t_, p, v));
  }

  inline void push_front(const T & t_, const double & p, const double & v, const double & a)
  {
    data.push_front(InterpolatorDataType<T>(t_, p, v, a));
  }

  inline void push_front(const T & t_, const double & p, bool boundary)
  {
    data.push_front(InterpolatorDataType<T>(t_, p, 0.0, 0.0, boundary));
  }

  inline void push_front(const T & t_, const double & p, const double & v, bool boundary)
  {
    data.push_front(InterpolatorDataType<T>(t_, p, v, 0.0, boundary));
  }

  inline void push_front(const T & t_, const double & p, const double & v, const double & a, bool boundary)
  {
    data.push_front(InterpolatorDataType<T>(t_, p, v, a, boundary));
  }

  void set(const int n, const T & t, const double & p);

  void set(const int n, const T & t, const double & p, const double & v);

  void set(const int n, const T & t, const double & p, const double & v, const double & a);

  virtual void set(const int n, const T & t, const double & p, bool boundary);

  virtual void set(const int n, const T & t, const double & p, const double & v, bool boundary);

  virtual void set(const int n, const T & t, const double & p, const double & v, const double & a, bool boundary);

  double getValue(T t_);

  void get(T t_, double & p);

  void get(T t_, double & p, double & v);

  virtual void get(T t_, double & p, double & v, double & a) = 0;

  virtual void update(void){};

  virtual void update(T t_){};

  virtual void update(T t_from, T t_to){};
};

template<typename T>
InterpolatorBase<T>::InterpolatorBase(const double & scale_, const double & eps_, interpolator_type ip_type_)
: ip_type(ip_type_), scale(scale_), eps(eps_)
{
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p)
{
  data[n].t = t_;
  data[n].pos = p;
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p, const double & v)
{
  data[n].t = t_;
  data[n].pos = p;
  data[n].vel = v;
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p, const double & v, const double & a)
{
  data[n].t = t_;
  data[n].pos = p;
  data[n].vel = v;
  data[n].acc = a;
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p, bool boundary)
{
  data[n].t = t_;
  data[n].pos = p;
  data[n].boundary = boundary;
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p, const double & v, bool boundary)
{
  data[n].t = t_;
  data[n].pos = p;
  data[n].vel = v;
  data[n].boundary = boundary;
}

template<typename T>
void InterpolatorBase<T>::set(int n, const T & t_, const double & p, const double & v, const double & a, bool boundary)
{
  data[n].t = t_;
  data[n].pos = p;
  data[n].vel = v;
  data[n].acc = a;
  data[n].boundary = boundary;
}

template<typename T>
void InterpolatorBase<T>::get(T t_, double & p)
{
  double v, a;
  get(t_, p, v, a);
}

template<typename T>
void InterpolatorBase<T>::get(T t_, double & p, double & v)
{
  double a;
  get(t_, p, v, a);
}

template<typename T>
double InterpolatorBase<T>::getValue(T t_)
{
  double p;
  get(t_, p);
  return p;
}
} // namespace motion_interpolator
} // namespace mc_planning
