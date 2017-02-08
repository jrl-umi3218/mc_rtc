#include <mc_control/log/Logger.h>

#include <mc_control/mc_global_controller.h>

#include <chrono>
#include <condition_variable>
#include <queue>
#include <iomanip>

namespace mc_control
{
  struct LoggerImpl
  {
    LoggerImpl(const bfs::path & directory, const std::string & tmpl)
    : builder_(1024*1024), directory(directory), tmpl(tmpl)
    {
    }

    virtual ~LoggerImpl() {}

    virtual void initialize(const bfs::path & path) = 0;
    virtual void write(uint8_t * data, int size) = 0;

    flatbuffers::FlatBufferBuilder builder_;

    bfs::path directory;
    std::string tmpl;
    double log_iter_ = 0;
    bool valid = true;
    std::ofstream log_;
  };

  namespace
  {
    struct LoggerNonThreadedPolicyImpl : public LoggerImpl
    {
      LoggerNonThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl)
      : LoggerImpl(directory, tmpl)
      {
      }

      virtual void initialize(const bfs::path & path) final
      {
        if(log_.is_open())
        {
          log_.close();
        }
        log_.open(path.string(), std::ofstream::binary);
      }

      virtual void write(uint8_t * data, int size) final
      {
        log_.write((char*)&size, sizeof(int));
        log_.write((char*)data, size);
      }
    };

    struct LoggerThreadedPolicyImpl : public LoggerImpl
    {
      LoggerThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl)
      : LoggerImpl(directory, tmpl)
      {
        log_sync_th_ = std::thread([this]()
        {
          while(log_sync_th_run_)
          {
            while(data_.size())
            {
              auto & d = data_.front();
              uint8_t * data = d.first;
              int size = d.second;
              log_.write((char*)&size, sizeof(int));
              log_.write((char*)data, size);
              delete[] data;
              data_.pop();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
          }
        }
        );
      }

      ~LoggerThreadedPolicyImpl()
      {
        log_sync_th_run_ = false;
        if(log_sync_th_.joinable())
        {
          log_sync_th_.join();
        }
      }

      virtual void initialize(const bfs::path & path) final
      {
        if(log_.is_open())
        {
          /* Wait until the previous log is flushed */
          while(data_.size())
          {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
          }
        }
        log_.open(path.string());
      }

      virtual void write(uint8_t * data, int size) final
      {
        uint8_t * ndata = new uint8_t[size];
        std::memcpy(ndata, data, size);
        data_.emplace(ndata, size);
      }

      std::thread log_sync_th_;
      bool log_sync_th_run_ = true;
      std::queue<std::pair<uint8_t*, int>> data_;
    };
  }

  Logger::Logger(const Policy & policy, const bfs::path & directory, const std::string & tmpl)
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

  Logger::~Logger()
  {
  }

  void Logger::start(const std::string & ctl_name, MCController * controller)
  {
    log_entries_.clear();
    auto get_log_path = [this, &ctl_name]()
    {
      std::stringstream ss;
      auto t = std::time(nullptr);
      auto tm = std::localtime(&t);
      ss << impl_->tmpl
         << "-" << ctl_name
         << "-" << (1900 + tm->tm_year)
         << "-" << std::setw(2) << std::setfill('0') << (1 + tm->tm_mon)
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_mday
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_hour
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_min
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_sec
         << ".bin";
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
      addLogEntry("t", [this, controller]()
                  {
                    impl_->log_iter_ += controller->timeStep;
                    return impl_->log_iter_ - controller->timeStep;
                  });
      addLogEntry("qIn", [controller]() -> const std::vector<double>&
                  {
                    return controller->robot().encoderValues();
                  });
      addLogEntry("ff", [controller]() -> const sva::PTransformd&
                  {
                    return controller->robot().mbc().bodyPosW[0];
                  });
      addLogEntry("qOut", [controller]()
                  {
                    const auto & qOut = controller->send(0).robots_state[0].q;
                    const auto & rjo = controller->robot().refJointOrder();
                    std::vector<double> ret(rjo.size(), 0);
                    for(size_t i = 0; i < rjo.size(); ++i)
                    {
                      const auto & jn = rjo[i];
                      if(qOut.count(jn))
                      {
                        ret[i] = qOut.at(jn)[0];
                      }
                    }
                    return ret;
                  });
      addLogEntry("tauIn", [controller]() -> const std::vector<double>&
                  {
                    return controller->robot().jointTorques();
                  });
      for(const auto & fs : controller->robot().forceSensors())
      {
        const auto & fs_name = fs.name();
        addLogEntry(fs.name(), [controller,fs_name]() -> const sva::ForceVecd&
                    {
                      return controller->robot().forceSensor(fs_name).wrench();
                    });
      }
      addLogEntry("pIn", [controller]() -> const Eigen::Vector3d&
                  {
                    return controller->robot().bodySensor().position();
                  });
      addLogEntry("rpyIn", [controller]() -> const Eigen::Quaterniond&
                  {
                    return controller->robot().bodySensor().orientation();
                  });
      addLogEntry("velIn", [controller]() -> const Eigen::Vector3d&
                  {
                    return controller->robot().bodySensor().linearVelocity();
                  });
      addLogEntry("rateIn", [controller]() -> const Eigen::Vector3d&
                  {
                    return controller->robot().bodySensor().angularVelocity();
                  });
      addLogEntry("accIn", [controller]() -> const Eigen::Vector3d&
                  {
                    return controller->robot().bodySensor().acceleration();
                  });
      impl_->log_iter_ = 0;
      impl_->valid = true;
    }
    else
    {
      impl_->valid = false;
      LOG_ERROR("Failed to open log file " << log_path)
    }
  }

  void Logger::log()
  {
    std::vector<std::string> keys;
    std::vector<uint8_t> types;
    std::vector<flatbuffers::Offset<void>> values;
    for(auto & e : log_entries_)
    {
      keys.push_back(e.first);
      e.second(impl_->builder_, types, values);
    }
    auto s_types = impl_->builder_.CreateVector(types);
    auto s_values = impl_->builder_.CreateVector(values);
    mc_control::log::LogBuilder log_builder(impl_->builder_);
    if(log_entries_changed_)
    {
      auto s_keys = impl_->builder_.CreateVectorOfStrings(keys);
      log_builder.add_keys(s_keys);
      log_entries_changed_ = false;
    }
    log_builder.add_values_type(s_types);
    log_builder.add_values(s_values);
    auto log = log_builder.Finish();
    impl_->builder_.Finish(log);
    int size = impl_->builder_.GetSize();
    uint8_t * data = impl_->builder_.GetBufferPointer();
    impl_->write(data, size);
    impl_->builder_.Clear();
  }

  void Logger::removeLogEntry(const std::string & name)
  {
    if(log_entries_.count(name))
    {
      log_entries_changed_ = true;
      log_entries_.erase(name);
    }
  }
}
