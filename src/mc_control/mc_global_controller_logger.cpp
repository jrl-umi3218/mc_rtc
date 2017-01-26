#include <mc_control/mc_global_controller.h>

#include <chrono>
#include <iomanip>

namespace mc_control
{
  struct MCGlobalController::LoggerImpl
  {
    LoggerImpl(const bfs::path & directory, const std::string & tmpl)
    : directory(directory), tmpl(tmpl)
    {
    }

    virtual ~LoggerImpl() {}

    virtual std::ofstream & initialize(const bfs::path & path) = 0;
    virtual std::ostream & get_stream() = 0;
    virtual void write() = 0;

    bfs::path directory;
    std::string tmpl;
    double log_iter_ = 0;
    bool valid = true;
  };

  namespace
  {
    struct LoggerNonThreadedPolicyImpl : public MCGlobalController::LoggerImpl
    {
      LoggerNonThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl)
      : MCGlobalController::LoggerImpl(directory, tmpl)
      {
      }

      virtual std::ofstream & initialize(const bfs::path & path) final
      {
        if(log_.is_open())
        {
          log_.close();
        }
        log_.open(path.string());
        return log_;
      }

      virtual std::ostream & get_stream() final
      {
        return log_;
      }

      virtual void write() final
      {
        log_ << std::endl;
      }

      std::ofstream log_;
    };

    struct LoggerThreadedPolicyImpl : public MCGlobalController::LoggerImpl
    {
      LoggerThreadedPolicyImpl(const bfs::path & directory, const std::string & tmpl)
      : MCGlobalController::LoggerImpl(directory, tmpl)
      {
        log_sync_th_ = std::thread([this]()
        {
          std::stringstream * tmp = nullptr;
          while(log_sync_th_run_)
          {
            while(log_sync_th_run_ && tmp == sync_ss_)
            {
              std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
            if(tmp != sync_ss_)
            {
              if(sync_ss_ != nullptr)
              {
                log_ << sync_ss_->str() << std::flush;
                sync_ss_->str("");
              }
              tmp = sync_ss_;
            }
          }
        }
        );
      }

      ~LoggerThreadedPolicyImpl()
      {
        std::string sync_ss_str = "";
        if(sync_ss_)
        {
          sync_ss_str = sync_ss_->str();
          sync_ss_ = nullptr;
        }
        log_sync_th_run_ = false;
        if(log_sync_th_.joinable())
        {
          log_sync_th_.join();
        }
        if(sync_ss_str.size())
        {
          log_ << sync_ss_str;
        }
        log_ << write_ss_->str();
      }

      virtual std::ofstream & initialize(const bfs::path & path) final
      {
        if(log_.is_open())
        {
          if(sync_ss_)
          {
            std::string sync_ss_str = sync_ss_->str();
            sync_ss_->str("");
            if(sync_ss_str.size())
            {
              log_ << sync_ss_str;
            }
          }
          std::string write_ss_str = write_ss_->str();
          write_ss_->str("");
          if(write_ss_str.size())
          {
            log_ << write_ss_str;
          }
          log_.close();
        }
        log_.open(path.string());
        log_ss_.str("");
        swap_ss_.str("");
        write_ss_ = &log_ss_;
        sync_ss_ = nullptr;
        log_sync_iter_ = 0;
        return log_;
      }

      virtual std::ostream & get_stream() final
      {
        return *write_ss_;
      }

      virtual void write() final
      {
        auto & log = *write_ss_;
        log << "\n";
        log_sync_iter_++;
        if(log_sync_iter_ == 500)
        {
          log_sync_iter_ = 0;
          if(sync_ss_)
          {
            std::swap(write_ss_, sync_ss_);
          }
          else
          {
            sync_ss_ = write_ss_;
            write_ss_ = &swap_ss_;
          }
        }
      }

      std::ofstream log_;
      std::stringstream log_ss_;
      std::stringstream swap_ss_;
      std::stringstream * write_ss_ = nullptr;
      std::stringstream * sync_ss_ = nullptr;
      std::thread log_sync_th_;
      bool log_sync_th_run_ = true;
      unsigned int log_sync_iter_ = 0;
    };
  }

  MCGlobalController::Logger::Logger(const Policy & policy, const bfs::path & directory, const std::string & tmpl)
  {
    switch(policy)
    {
      case Policy::NON_THREADED:
        impl.reset(new LoggerNonThreadedPolicyImpl(directory, tmpl));
        break;
      case Policy::THREADED:
        impl.reset(new LoggerThreadedPolicyImpl(directory, tmpl));
        break;
    };
  }

  MCGlobalController::Logger::~Logger()
  {
  }

  void MCGlobalController::Logger::log_header(const std::string & ctl_name, MCController * controller)
  {
    auto get_log_path = [this, &ctl_name]()
    {
      std::stringstream ss;
      auto t = std::time(nullptr);
      auto tm = std::localtime(&t);
      ss << impl->tmpl
         << "-" << ctl_name
         << "-" << (1900 + tm->tm_year)
         << "-" << std::setw(2) << std::setfill('0') << (1 + tm->tm_mon)
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_mday
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_hour
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_min
         << "-" << std::setw(2) << std::setfill('0') << tm->tm_sec
         << ".log";
      bfs::path log_path = impl->directory / bfs::path(ss.str().c_str());
      LOG_INFO("Will log controller outputs to " << log_path)
      return log_path;
    };
    auto log_path = get_log_path();
    auto & log = impl->initialize(log_path);
    std::stringstream ss_sym;
    ss_sym << impl->tmpl << "-" << ctl_name << "-latest.log";
    bfs::path log_sym_path = impl->directory / bfs::path(ss_sym.str().c_str());
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
    if(log.is_open())
    {
      log << "t";
      for(unsigned int i = 0; i < static_cast<unsigned int>(controller->robot().encoderValues().size()); ++i)
      {
        log << ";qIn" << i;
      }
      log << ";ff_qw;ff_qx;ff_qy;ff_qz;ff_tx;ff_ty;ff_tz";
      for(unsigned int i = 0; i < static_cast<unsigned int>(controller->robot().refJointOrder().size()); ++i)
      {
        log << ";qOut" << i;
      }
      for(unsigned int i = 0; i < static_cast<unsigned int>(controller->robot().jointTorques().size()); ++i)
      {
        log << ";taucIn" << i;
      }
      for(const auto & fs : controller->robot().forceSensors())
      {
        const auto& wn = fs.name();
        log << ";" << wn << "_fx";
        log << ";" << wn << "_fy";
        log << ";" << wn << "_fz";
        log << ";" << wn << "_cx";
        log << ";" << wn << "_cy";
        log << ";" << wn << "_cz";
      }
      log << ";" << "p_x";
      log << ";" << "p_y";
      log << ";" << "p_z";
      log << ";" << "rpy_r";
      log << ";" << "rpy_p";
      log << ";" << "rpy_y";
      log << ";" << "vel_x";
      log << ";" << "vel_y";
      log << ";" << "vel_z";
      log << ";" << "rate_x";
      log << ";" << "rate_y";
      log << ";" << "rate_z";
      log << ";" << "acc_x";
      log << ";" << "acc_y";
      log << ";" << "acc_z";

      controller->log_header(log);
      log << std::endl;
      impl->log_iter_ = 0;
      impl->valid = true;
    }
    else
    {
      impl->valid = false;
      LOG_ERROR("Failed to open log file " << log_path)
    }
  }

  void MCGlobalController::Logger::log_data(MCGlobalController & gc, MCController * controller)
  {
    auto & log = impl->get_stream();
    log << impl->log_iter_;
    impl->log_iter_ += gc.timestep();
    for(const auto & qi : controller->robot().encoderValues())
    {
      log << ";" << qi;
    }
    const auto & ff = controller->robot().mbc().q[0];
    for(const auto & ffi : ff)
    {
      log << ";" << ffi;
    }
    const auto & qOut = gc.send(impl->log_iter_).robots_state[0].q;
    for(const auto & jn : gc.ref_joint_order())
    {
      if(qOut.count(jn))
      {
        log << ";" << qOut.at(jn)[0];
      }
      else
      {
        log << ";" << 0;
      }
    }
    for(const auto & ti : controller->robot().jointTorques())
    {
      log << ";" << ti;
    }
    for(const auto & fs : controller->robot().forceSensors())
    {
      log << ";" << fs.wrench().force().x();
      log << ";" << fs.wrench().force().y();
      log << ";" << fs.wrench().force().z();
      log << ";" << fs.wrench().couple().x();
      log << ";" << fs.wrench().couple().y();
      log << ";" << fs.wrench().couple().z();
    }
    const auto & pIn = controller->robot().bodySensor().position();
    log << ";" << pIn.x();
    log << ";" << pIn.y();
    log << ";" << pIn.z();

    const auto & rpyIn = controller->robot().bodySensor().orientation();
    log << ";" << rpyIn.x();
    log << ";" << rpyIn.y();
    log << ";" << rpyIn.z();

    const auto & velIn = controller->robot().bodySensor().linearVelocity();
    log << ";" << velIn.x();
    log << ";" << velIn.y();
    log << ";" << velIn.z();

    const auto & rateIn = controller->robot().bodySensor().angularVelocity();
    log << ";" << rateIn.x();
    log << ";" << rateIn.y();
    log << ";" << rateIn.z();

    const auto & accIn = controller->robot().bodySensor().acceleration();
    log << ";" << accIn.x();
    log << ";" << accIn.y();
    log << ";" << accIn.z();

    controller->log_data(log);

    impl->write();
  }
}
