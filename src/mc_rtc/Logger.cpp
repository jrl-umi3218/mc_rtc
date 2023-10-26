/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/Logger.h>
#include <mc_rtc/utils.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <chrono>
#include <fstream>
#include <iomanip>
#include <thread>

namespace mc_rtc
{

const uint8_t Logger::magic[4] = {0x41, 0x4e, 0x4e, 0x45};

const uint8_t Logger::version = 1;

struct LoggerImpl
{
  LoggerImpl(const std::string & directory, const std::string & tmpl)
  : data_(1024 * 1024), directory(directory), tmpl(tmpl)
  {
  }

  virtual ~LoggerImpl() {}

  virtual void initialize(const bfs::path & path) = 0;
  virtual void write(char * data, size_t size) = 0;
  virtual void flush() {}

  std::vector<char> data_;

  bfs::path directory;
  std::string tmpl;
  double log_iter_ = 0;
  bool valid_ = true;
  std::string path_ = "";
  std::ofstream log_;

protected:
  inline void fwrite(char * data, uint64_t size)
  {
    log_.write((char *)&size, sizeof(uint64_t));
    log_.write(data, static_cast<int>(size));
  }

  // Open file and write magic number to it right away
  void open(const std::string & path)
  {
    path_ = path;
    log_.open(path, std::ofstream::binary);
    static_assert(sizeof(uint8_t) == sizeof(char));
    log_.write((const char *)&Logger::magic, sizeof(Logger::magic) - sizeof(uint8_t));
    const char version = static_cast<uint8_t>(Logger::magic[3] + Logger::version);
    log_.write(&version, sizeof(uint8_t));
  }
};

namespace
{
struct LoggerNonThreadedPolicyImpl : public LoggerImpl
{
  LoggerNonThreadedPolicyImpl(const std::string & directory, const std::string & tmpl) : LoggerImpl(directory, tmpl) {}

  void initialize(const bfs::path & path) final
  {
    if(log_.is_open()) { log_.close(); }
    open(path.string());
  }

  void write(char * data, size_t size) final
  {
    if(valid_) { fwrite(data, size); }
  }

  void flush() final
  {
    if(valid_) { log_.flush(); }
  }
};

struct LoggerThreadedPolicyImpl : public LoggerImpl
{
  LoggerThreadedPolicyImpl(const std::string & directory, const std::string & tmpl) : LoggerImpl(directory, tmpl)
  {
    log_sync_th_ = std::thread(
        [this]()
        {
          while(log_sync_th_run_ && valid_)
          {
            while(!write_data()) {}
            std::this_thread::sleep_for(std::chrono::microseconds(500));
          }
          while(!write_data()) {}
        });
  }

  ~LoggerThreadedPolicyImpl()
  {
    log_sync_th_run_ = false;
    if(log_sync_th_.joinable()) { log_sync_th_.join(); }
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

  void initialize(const bfs::path & path) final
  {
    if(log_.is_open())
    {
      /* Wait until the previous log is flushed */
      while(!data_.empty()) { std::this_thread::sleep_for(std::chrono::microseconds(500)); }
      log_.close();
    }
    open(path.string());
  }

  void write(char * data, size_t size) final
  {
    char * ndata = new char[size];
    std::memcpy(ndata, data, size);
    if(!data_.push({ndata, size}))
    {
      mc_rtc::log::critical("Data cannot be added to the log");
      delete[] ndata;
    }
  }

  std::thread log_sync_th_;
  bool log_sync_th_run_ = true;
  CircularBuffer<std::pair<char *, size_t>, 2048> data_;
  std::pair<char *, size_t> pop_;
};
} // namespace

Logger::Logger(const Policy & policy, const std::string & directory, const std::string & tmpl)
{
  setup(policy, directory, tmpl);
}

Logger::~Logger() {}

void Logger::setup(const Policy & policy, const std::string & directory, const std::string & tmpl)
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

auto Logger::find_entry(const std::string & name) -> std::vector<LogEntry>::iterator
{
  return std::find_if(log_entries_.begin(), log_entries_.end(), [&](const auto & e) { return e.key == name; });
}

void Logger::start(const std::string & ctl_name, double timestep, bool resume, double start_t)
{
  auto get_log_path = [this, &ctl_name]()
  {
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
    log::info("Will log controller outputs to {}", log_path.string());
    return log_path;
  };
  auto log_path = get_log_path();
  impl_->initialize(log_path);
  std::stringstream ss_sym;
  ss_sym << impl_->tmpl << "-" << ctl_name << "-latest.bin";
  bfs::path log_sym_path = impl_->directory / bfs::path(ss_sym.str().c_str());
  if(bfs::is_symlink(log_sym_path)) { bfs::remove(log_sym_path); }
  if(!bfs::exists(log_sym_path))
  {
    boost::system::error_code ec;
    bfs::create_symlink(log_path, log_sym_path, ec);
    if(!ec) { log::info("Updated latest log symlink: {}", log_sym_path.string()); }
    else { log::info("Failed to create latest log symlink: {}", ec.message()); }
  }
  if(impl_->log_.is_open())
  {
    if(resume)
    {
      // Re-create key events based on the current set of entries
      log_events_.clear();
      for(const auto & e : log_entries_) { log_events_.push_back(KeyAddedEvent{e.type, e.key}); }
    }
    else { impl_->log_iter_ = start_t; }
    if(find_entry("t") == log_entries_.end())
    {
      addLogEntry("t", this,
                  [this, timestep]()
                  {
                    impl_->log_iter_ += timestep;
                    return impl_->log_iter_ - timestep;
                  });
    }
    impl_->valid_ = true;
  }
  else
  {
    impl_->valid_ = false;
    log::error("Failed to open log file {}", log_path.string());
  }
  log_events_.push_back(StartEvent{});
}

void Logger::open(const std::string & file, double timestep, double start_t)
{
  impl_->initialize(file);
  if(impl_->log_.is_open())
  {
    if(find_entry("t") == log_entries_.end())
    {
      addLogEntry("t", this,
                  [this, timestep]()
                  {
                    impl_->log_iter_ += timestep;
                    return impl_->log_iter_ - timestep;
                  });
    }
    impl_->log_iter_ = start_t;
    impl_->valid_ = true;
  }
  else
  {
    impl_->valid_ = false;
    log::error("Failed to open log file {}", file);
  }
  log_events_.push_back(StartEvent{});
}

void Logger::log()
{
  mc_rtc::MessagePackBuilder builder(impl_->data_);
  builder.start_array(2);
  if(log_events_.size())
  {
    builder.start_array(log_events_.size());
    auto event_visitor = [&builder, this](auto && event)
    {
      using T = std::decay_t<decltype(event)>;
      if constexpr(std::is_same_v<T, KeyAddedEvent>)
      {
        builder.start_array(3);
        builder.write(static_cast<uint8_t>(0));
        builder.write(static_cast<typename std::underlying_type<log::LogType>::type>(event.type));
        builder.write(event.key);
        builder.finish_array();
      }
      else if constexpr(std::is_same_v<T, KeyRemovedEvent>)
      {
        builder.start_array(2);
        builder.write(static_cast<uint8_t>(1));
        builder.write(event.key);
        builder.finish_array();
      }
      else if constexpr(std::is_same_v<T, GUIEvent>)
      {
        builder.start_array(4);
        builder.write(static_cast<uint8_t>(2));
        builder.write(event.category);
        builder.write(event.name);
        builder.write(event.data);
        builder.finish_array();
      }
      else if constexpr(std::is_same_v<T, StartEvent>)
      {
        builder.start_array(7);
        builder.write(static_cast<uint8_t>(3));
        builder.write(meta_.timestep);
        builder.write(meta_.main_robot);
        builder.write(meta_.main_robot_module);
        builder.write(meta_.init);
        builder.write(meta_.init_q);
        builder.write(meta_.calibs);
        builder.finish_array();
      }
      else { static_assert(!std::is_same_v<T, T>, "non-exhaustive visitor"); }
    };

    for(auto & e : log_events_) { std::visit(event_visitor, e); }
    builder.finish_array();
    log_events_.resize(0);
  }
  else { builder.write(); }
  builder.start_array(log_entries_.size());
  for(auto & e : log_entries_) { e.log_cb(builder); }
  builder.finish_array();
  builder.finish_array();
  size_t s = builder.finish();
  impl_->write(impl_->data_.data(), s);
}

void Logger::removeLogEntry(const std::string & name)
{
  auto it = find_entry(name);
  if(it != log_entries_.end())
  {
    log_events_.push_back(KeyRemovedEvent{name});
    log_entries_.erase(it);
  }
}

void Logger::removeLogEntries(const void * source)
{
  for(auto it = log_entries_.begin(); it != log_entries_.end();)
  {
    if(it->source == source)
    {
      log_events_.push_back(KeyRemovedEvent{it->key});
      it = log_entries_.erase(it);
    }
    else { ++it; }
  }
}

void Logger::clear(bool record)
{
  for(auto it = log_entries_.begin(); it != log_entries_.end();)
  {
    if(it->key != "t")
    {
      if(record) { log_events_.push_back(KeyRemovedEvent{it->key}); }
      it = log_entries_.erase(it);
    }
    else { ++it; }
  }
}

double Logger::t() const
{
  return impl_->log_iter_;
}

const std::string & Logger::path() const
{
  return impl_->path_;
}

void Logger::flush()
{
  impl_->flush();
}

} // namespace mc_rtc
