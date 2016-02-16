#include <string>
#include <unordered_map>
#include <vector>

#include <mc_rtc/config.h>

namespace pdgains
{

inline bool setPGain(int joint, double pgain);
inline bool setPGain(const std::string & jname, double pgain);
inline bool setDGain(int joint, double dgain);
inline bool setDGain(const std::string & jname, double dgain);
inline bool setPGains(const std::vector<int> & joints, const std::vector<double> & pgains);
inline bool setPGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains);
inline bool setDGains(const std::vector<int> & joints, const std::vector<double> & dgains);
inline bool setDGains(const std::vector<std::string> & jnames, const std::vector<double> & dgains);
inline bool setPDGains(const std::vector<int> & joints, const std::vector<double> & pgains, const std::vector<double> & dgains);
inline bool setPDGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains, const std::vector<double> & dgains);
inline bool setPDGains(const std::vector<double> & pgains, const std::vector<double> & dgains);
inline bool getPGain(int joint, double &pgain);
inline bool getPGain(const std::string & jname, double & pgain);
inline bool getPGains(std::vector<double> &pgains);
inline bool getDGain(int joint, double &dgain);
inline bool getDGain(const std::string & jname, double & dgain);
inline bool getDGains(std::vector<double> &dgains);

}

#ifdef MC_RTC_HAS_HRPSYS_BASE
#include <hrpsys/io/iob.h>
#endif

#ifdef MC_RTC_HAS_HRPSYS_BASE
namespace hrp2
{
  static const std::unordered_map<std::string, int> joints =
  {
    {"RLEG_JOINT0",0},
    {"RLEG_JOINT1",1},
    {"RLEG_JOINT2",2},
    {"RLEG_JOINT3",3},
    {"RLEG_JOINT4",4},
    {"RLEG_JOINT5",5},
    {"LLEG_JOINT0",6},
    {"LLEG_JOINT1",7},
    {"LLEG_JOINT2",8},
    {"LLEG_JOINT3",9},
    {"LLEG_JOINT4",10},
    {"LLEG_JOINT5",11},
    {"CHEST_JOINT0",12},
    {"CHEST_JOINT1",13},
    {"HEAD_JOINT0",14},
    {"HEAD_JOINT1",15},
    {"RARM_JOINT0",16},
    {"RARM_JOINT1",17},
    {"RARM_JOINT2",18},
    {"RARM_JOINT3",19},
    {"RARM_JOINT4",20},
    {"RARM_JOINT5",21},
    {"RARM_JOINT6",22},
    {"RARM_JOINT7",23},
    {"LARM_JOINT0",24},
    {"LARM_JOINT1",25},
    {"LARM_JOINT2",26},
    {"LARM_JOINT3",27},
    {"LARM_JOINT4",28},
    {"LARM_JOINT5",29},
    {"LARM_JOINT6",30},
    {"LARM_JOINT7",31}
  };
}

namespace pdgains
{

inline bool setPGain(int joint, double pgain)
{
  bool b = false;
  if(open_iob())
  {
    b = write_pgain(joint, pgain);
    close_iob();
  }
  return b;
}

inline bool setPGain(const std::string & jname, double pgain)
{
  return setPGain(hrp2::joints.at(jname), pgain);
}

inline bool setDGain(int joint, double dgain)
{
  bool b = false;
  if(open_iob())
  {
    b = write_dgain(joint, dgain);
    close_iob();
  }
  return b;
}

inline bool setDGain(const std::string & jname, double dgain)
{
  return setDGain(hrp2::joints.at(jname), dgain);
}

inline bool setPGains(const std::vector<int> & joints, const std::vector<double> & pgains)
{
  if(joints.size() != pgains.size()) { return false; }
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < joints.size(); ++i)
    {
      b = write_pgain(joints[i], pgains[i]);
    }
    close_iob();
  }
  return b;
}

inline bool setPGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains)
{
  std::vector<int> joints;
  for(const auto & jn : jnames)
  {
    joints.push_back(hrp2::joints.at(jn));
  }
  return setPGains(joints, pgains);
}

inline bool setDGains(const std::vector<int> & joints, const std::vector<double> & dgains)
{
  if(joints.size() != dgains.size()) { return false; }
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < joints.size(); ++i)
    {
      b = write_dgain(joints[i], dgains[i]);
    }
    close_iob();
  }
  return b;
}

inline bool setDGains(const std::vector<std::string> & jnames, const std::vector<double> & dgains)
{
  std::vector<int> joints;
  for(const auto & jn : jnames)
  {
    joints.push_back(hrp2::joints.at(jn));
  }
  return setDGains(joints, dgains);
}

inline bool setPDGains(const std::vector<int> & joints, const std::vector<double> & pgains, const std::vector<double> & dgains)
{
  if(joints.size() != pgains.size() || joints.size() != dgains.size()) { return false; }
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < joints.size(); ++i)
    {
      b = write_pgain(joints[i], pgains[i]) && b;
      b = write_dgain(joints[i], dgains[i]) && b;
    }
    close_iob();
  }
  return b;
}

inline bool setPDGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains, const std::vector<double> & dgains)
{
  std::vector<int> joints;
  for(const auto & jn : jnames)
  {
    joints.push_back(hrp2::joints.at(jn));
  }
  return setPDGains(joints, pgains, dgains);
}

inline bool setPDGains(const std::vector<double> & pgains, const std::vector<double> & dgains)
{
  if(pgains.size() != dgains.size()) { return false; }
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < pgains.size(); ++i)
    {
      b = write_pgain(static_cast<int>(i), pgains[i]) && b;
      b = write_dgain(static_cast<int>(i), dgains[i]) && b;
    }
    close_iob();
  }
  return b;
}

inline bool getPGain(int joint, double &pgain)
{
  bool b = false;
  if(open_iob())
  {
    b = read_pgain(joint, &pgain);
    close_iob();
  }
  return b;
}

inline bool getPGain(const std::string & jname, double & pgain)
{
  return getPGain(hrp2::joints.at(jname), pgain);
}

inline bool getPGains(std::vector<double> &pgains)
{
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < pgains.size(); ++i)
    {
      b = read_pgain(static_cast<int>(i), &(pgains[i]));
    }
    close_iob();
  }
  return b;
}

inline bool getDGain(int joint, double &dgain)
{
  bool b = false;
  if(open_iob())
  {
    b = read_dgain(joint, &dgain);
    close_iob();
  }
  return b;
}

inline bool getDGain(const std::string & jname, double & dgain)
{
  return getDGain(hrp2::joints.at(jname), dgain);
}

inline bool getDGains(std::vector<double> &dgains)
{
  bool b = false;
  if(open_iob())
  {
    for(size_t i = 0; i < dgains.size(); ++i)
    {
      b = read_dgain(static_cast<int>(i), &(dgains[i]));
    }
    close_iob();
  }
  return b;
}

} // namespace pdgains
#else // isdef MC_RTC_HAS_HRPSYS_BASE

namespace pdgains
{
inline bool setPGain(int joint, double pgain) { return true; }
inline bool setPGain(const std::string & jname, double pgain) { return true; }
inline bool setDGain(int joint, double dgain) { return true; }
inline bool setDGain(const std::string & jname, double dgain) { return true; }
inline bool setPGains(const std::vector<int> & joints, const std::vector<double> & pgains) { return true; }
inline bool setPGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains) { return true; }
inline bool setDGains(const std::vector<int> & joints, const std::vector<double> & dgains) { return true; }
inline bool setDGains(const std::vector<std::string> & jnames, const std::vector<double> & dgains) { return true; }
inline bool setPDGains(const std::vector<int> & joints, const std::vector<double> & pgains, const std::vector<double> & dgains) { return true; }
inline bool setPDGains(const std::vector<std::string> & jnames, const std::vector<double> & pgains, const std::vector<double> & dgains) { return true; }
inline bool setPDGains(const std::vector<double> & pgains, const std::vector<double> & dgains) { return true; }
inline bool getPGain(int joint, double &pgain) { return true; }
inline bool getPGain(const std::string & jname, double & pgain) { return true; }
inline bool getPGains(std::vector<double> &pgains) { return true; }
inline bool getDGain(int joint, double &dgain) { return true; }
inline bool getDGain(const std::string & jname, double & dgain) { return true; }
inline bool getDGains(std::vector<double> &dgains) { return true; }
} // namespace pdgains
#endif // MC_RTC_HAS_HRPSYS_BASE
