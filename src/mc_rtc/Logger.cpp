/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/Logger.h>
#include <mc_rtc/utils.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <thread>

namespace mc_rtc
{

const uint8_t Logger::magic[4] = {0x41, 0x4e, 0x4e, 0x45};

struct LoggerImpl
{
  LoggerImpl(const bfs::path & directory, const std::string & tmpl)
  : data_(1024 * 1024), directory(directory), tmpl(tmpl)
  {
  }

  virtual ~LoggerImpl() {}

  virtual void initialize(const bfs::path & path) = 0;
  virtual void write(char * data, size_t size) = 0;

  std::vector<char> data_;

  bfs::path directory;
  std::string tmpl;
  double log_iter_ = 0;
  bool valid_ = true;
  std::ofstream log_;

protected:
  inline void fwrite(char * data, size_t size)
  {
    log_.write((char *)&size, sizeof(size_t));
    log_.write(data, static_cast<int>(size));
  }

  // Open file and write magic number to it right away
  void open(const std::string & path)
  {
    log_.open(path, std::ofstream::binary);
    log_.write((const char *)&Logger::magic, sizeof(Logger::magic));
  }
};

namespace
{
struct LoggerNonThreadedPolicyImpl : public LoggerImpl
{
  LoggerNonThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl) : LoggerImpl(directory, tmpl) {}

  virtual void initialize(const bfs::path & path) final
  {
    if(log_.is_open())
    {
      log_.close();
    }
    open(path.string());
  }

  virtual void write(char * data, size_t size) final
  {
    if(valid_)
    {
      fwrite(data, size);
    }
  }
};

struct LoggerThreadedPolicyImpl : public LoggerImpl
{
  LoggerThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl) : LoggerImpl(directory, tmpl)
  {
    log_sync_th_ = std::thread([this]() {
      while(log_sync_th_run_ && valid_)
      {
        while(!write_data())
          ;
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
      while(!write_data())
        ;
    });
  }

  ~LoggerThreadedPolicyImpl()
  {
    log_sync_th_run_ = false;
    if(log_sync_th_.joinable())
    {
      log_sync_th_.join();
    }
  }

  // Returns true when all data has been consumed
  bool write_data()
  {
    if(data_.pop(pop_))
    {
      char * data = pop_.first;
      size_t size = pop_.second;
      fwrite(data, size);
      delete[] data;
      return false;
    }
    return true;
  }

  virtual void initialize(const bfs::path & path) final
  {
    if(log_.is_open())
    {
      /* Wait until the previous log is flushed */
      while(!data_.empty())
      {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
      log_.close();
    }
    open(path.string());
  }

  virtual void write(char * data, size_t size) final
  {
    char * ndata = new char[size];
    std::memcpy(ndata, data, size);
    data_.push({ndata, size});
  }

  std::thread log_sync_th_;
  bool log_sync_th_run_ = true;
  CircularBuffer<std::pair<char *, size_t>, 512> data_;
  std::pair<char *, size_t> pop_;
};
} // namespace

Logger::Logger(const Policy & policy, const bfs::path & directory, const std::string & tmpl)
{
  setup(policy, directory, tmpl);
}

Logger::~Logger() {}

void Logger::setup(const Policy & policy, const bfs::path & directory, const std::string & tmpl)
{
  switch(policy)
  {
    case Policy::NON_THREADED:
      impl_.reset(new LoggerNonThreadedPolicyImpl(directory, tmpl));
      break;
    case Policy::THREADED:
      impl_.reset(new LoggerThreadedPolicyImpl(directory, tmpl));
      break;
  };
}

void Logger::start(const std::string & ctl_name, double timestep, bool resume)
{
  auto get_log_path = [this, &ctl_name]() {
    std::stringstream ss;
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    // clang-format off
    ss << impl_->tmpl
       << "-" << ctl_name
       << "-" << (1900 + tm->tm_year)
       << "-" << std::setw(2) << std::setfill('0') << (1 + tm->tm_mon)
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_mday
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_hour
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_min
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_sec
       << ".bin";
    // clang-format on
    bfs::path log_path = impl_->directory / bfs::path(ss.str().c_str());
    LOG_INFO("Will log controller outputs to " << log_path)
    return log_path;
  };
  auto log_path = get_log_path();
  impl_->initialize(log_path);
  std::stringstream ss_sym;
  ss_sym << impl_->tmpl << "-" << ctl_name << "-latest.bin";
  bfs::path log_sym_path = impl_->directory / bfs::path(ss_sym.str().c_str());
  if(bfs::is_symlink(log_sym_path))
  {
    bfs::remove(log_sym_path);
  }
  if(!bfs::exists(log_sym_path))
  {
    boost::system::error_code ec;
    bfs::create_symlink(log_path, log_sym_path, ec);
    if(!ec)
    {
      LOG_INFO("Updated latest log symlink: " << log_sym_path)
    }
    else
    {
      LOG_INFO("Failed to create latest log symlink: " << ec.message())
    }
  }
  if(impl_->log_.is_open())
  {
    if(!log_entries_.count("t"))
    {
      addLogEntry("t", [this, timestep]() {
        impl_->log_iter_ += timestep;
        return impl_->log_iter_ - timestep;
      });
    }
    if(!resume)
    {
      impl_->log_iter_ = 0;
    }
    impl_->valid_ = true;
  }
  else
  {
    impl_->valid_ = false;
    LOG_ERROR("Failed to open log file " << log_path)
  }
  // Force rewrite of the log header
  log_entries_changed_ = true;
}

void Logger::log()
{
  mc_rtc::MessagePackBuilder builder(impl_->data_);
  builder.start_array(2);
  if(log_entries_changed_)
  {
    builder.start_array(log_entries_.size());
    for(auto & e : log_entries_)
    {
      builder.write(e.first);
    }
    builder.finish_array();
    log_entries_changed_ = false;
  }
  else
  {
    builder.write();
  }
  builder.start_array(2 * log_entries_.size());
  for(auto & e : log_entries_)
  {
    e.second(builder);
  }
  builder.finish_array();
  builder.finish_array();
  size_t s = builder.finish();
  impl_->write(impl_->data_.data(), s);
}

void Logger::removeLogEntry(const std::string & name)
{
  if(log_entries_.count(name))
  {
    log_entries_changed_ = true;
    log_entries_.erase(name);
  }
}

double Logger::t() const
{
  return impl_->log_iter_;
}
} // namespace mc_rtc
