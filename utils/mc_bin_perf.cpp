#pragma once

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>

#include <array>
#include <iomanip>
#include <iostream>

struct PrettyColumn
{
  PrettyColumn(const std::string & name, int prec) : name_(name), prec_(prec), max_size_(name_.size()) {}
  PrettyColumn(const std::string & name) : PrettyColumn(name, 3) {}

  template<typename T>
  void put(const T & data)
  {
    std::stringstream ss;
    ss << std::setprecision(prec_) << data;
    data_.push_back(ss.str());
    max_size_ = std::max(max_size_, data_.back().size());
  }

  std::string name_;
  std::vector<std::string> data_;
  int prec_;
  size_t max_size_ = 0;
};

template<size_t N>
struct PrettyTable
{
  PrettyTable(const std::array<PrettyColumn, N> & columns) : columns_(columns) {}

  template<typename... Args>
  void put(const std::tuple<Args...> & args)
  {
    put_impl(args);
  }

  template<size_t i = 0, typename... Args, typename std::enable_if<i >= N, int>::type = 0>
  void put_impl(const std::tuple<Args...> &)
  {
  }

  template<size_t i = 0,
           typename... Args,
           typename std::enable_if<i<N, int>::type = 0> void put_impl(const std::tuple<Args...> & args)
  {
    columns_[i].put(std::get<i>(args));
    put_impl<i + 1>(args);
  }

  size_t size()
  {
    size_t s = N + 1;
    for(size_t i = 0; i < N; ++i)
    {
      s += columns_[i].max_size_ + 2;
    }
    return s;
  }

  template<typename Stream>
  void print(Stream & os)
  {
    auto s = size();
    os << std::string(s, '-') << "\n";
    os << '|';
    for(size_t i = 0; i < N; ++i)
    {
      auto & col = columns_[i];
      os << ' ' << std::setw(col.max_size_) << std::setfill(' ') << col.name_ << " |";
    }
    os << '\n' << std::string(s, '-') << '\n';
    for(size_t i = 0; i < columns_[0].data_.size(); ++i)
    {
      os << '|';
      for(size_t j = 0; j < N; ++j)
      {
        auto & col = columns_[j];
        os << ' ' << std::setw(col.max_size_) << std::setfill(' ') << col.data_[i] << " |";
      }
      os << '\n';
    }
    os << std::string(s, '-') << "\n";
  }

private:
  std::array<PrettyColumn, N> columns_;
};

using PerfTable = PrettyTable<5>;

/** This utilty works with a binary log and outputs performance information */

static const std::string match = "perf_";

void usage(char * name)
{
  std::cerr << name << " [log] [entry = t]\n";
}

std::pair<size_t, size_t> getRange(const mc_rtc::log::FlatLog & log, const std::string & key)
{
  if(!log.has(key))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No entry named " << key << " in log")
  }
  size_t start = 0;
  size_t end = 0;
  auto t = log.getRaw<double>(key);
  while(t[start] == nullptr)
  {
    start++;
  }
  end = start;
  while(t[end] != nullptr && end < t.size())
  {
    end++;
  }
  if(start == end)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, key << " does not look like a valid time entry")
  }
  return {start, end};
}

void show_perf(const mc_rtc::log::FlatLog & log,
               const std::pair<size_t, size_t> & range,
               const std::string & key,
               PerfTable & vt)
{
  auto data = log.get<double>(key);
  /** Based on https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm */
  double max = data[range.first];
  double min = max != 0 ? max : std::numeric_limits<double>::infinity();
  double prev_avg = max;
  double avg = 0;
  double std_sq = 0;
  double M_2_i = 0;
  double N = 1;
  for(size_t i = range.first + 1; i < range.second; ++i)
  {
    if(data[i] != 0)
    {
      N++;
      avg = prev_avg + (data[i] - prev_avg) / N;
      M_2_i = M_2_i + (data[i] - prev_avg) * (data[i] - avg);
      std_sq = M_2_i / N;
      prev_avg = avg;
      max = std::max(max, data[i]);
      min = std::min(min, data[i]);
    }
  }
  double std = std::sqrt(std_sq);
  auto row = std::make_tuple(key.substr(match.size()), avg, std, min, max);
  vt.put(row);
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage(argv[0]);
    return 1;
  }
  std::string file = argv[1];
  std::string key = "t";
  if(argc > 2)
  {
    key = argv[2];
  }
  PerfTable vt(std::array<PrettyColumn, 5>{PrettyColumn{""}, PrettyColumn{"Average"}, PrettyColumn{"StdEv"},
                                           PrettyColumn{"Min"}, PrettyColumn{"Max"}});
  mc_rtc::log::FlatLog log(file);
  auto range = getRange(log, key);
  auto keys = log.entries();
  for(const auto & k : keys)
  {
    if(k.find(match) == 0)
    {
      show_perf(log, range, k, vt);
    }
  }
  vt.print(std::cout);
  return 0;
}
