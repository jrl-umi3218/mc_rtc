/** This file holds all mc_rtc::Configuration based serialization operation for mc_rbdyn objects */

#include <mc_rtc/Configuration.h>

#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Collision.h>

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>

#include <mc_rtc/logging.h>

namespace mc_rtc
{
  template<>
  struct ConfigurationLoader<sva::PTransformd>
  {
    static sva::PTransformd load(const mc_rtc::Configuration & config)
    {
      Eigen::Matrix3d r = config("rotation");
      return {r, config("translation")};
    }

    static mc_rtc::Configuration save(const sva::PTransformd & pt)
    {
      mc_rtc::Configuration config;
      config.add("translation", pt.translation());
      config.add("rotation", pt.rotation());
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<rbd::Joint::Type>
  {
    static rbd::Joint::Type load(const mc_rtc::Configuration & config)
    {
      std::string type = "";
      config("type", type);
      if(type == "rev")
      {
        return rbd::Joint::Type::Rev;
      }
      if(type == "prism")
      {
        return rbd::Joint::Type::Prism;
      }
      if(type == "spherical")
      {
        return rbd::Joint::Type::Spherical;
      }
      if(type == "planar")
      {
        return rbd::Joint::Type::Planar;
      }
      if(type == "cylindrical")
      {
        return rbd::Joint::Type::Cylindrical;
      }
      if(type == "free")
      {
        return rbd::Joint::Type::Free;
      }
      if(type == "fixed")
      {
        return rbd::Joint::Type::Fixed;
      }
      LOG_ERROR(type << " was stored as joint type, cannot comprehend that")
      throw(std::runtime_error("Invalid joint type stored"));
    }

    static mc_rtc::Configuration save(const rbd::Joint::Type & type)
    {
      mc_rtc::Configuration config;
      std::string typeStr = "";
      switch(type)
      {
        case rbd::Joint::Type::Rev:
          typeStr = "rev";
          break;
        case rbd::Joint::Type::Prism:
          typeStr = "prism";
          break;
        case rbd::Joint::Type::Spherical:
          typeStr = "spherical";
          break;
        case rbd::Joint::Type::Planar:
          typeStr = "planar";
          break;
        case rbd::Joint::Type::Cylindrical:
          typeStr = "cylindrical";
          break;
        case rbd::Joint::Type::Free:
          typeStr = "free";
          break;
        case rbd::Joint::Type::Fixed:
          typeStr = "fixed";
          break;
        default:
          LOG_ERROR("Cannot serialize joint type " << type)
          throw(std::runtime_error("Invalid joint type to save"));
      }
      config.add("type", typeStr);
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<mc_rbdyn::Base>
  {
    static mc_rbdyn::Base load(const mc_rtc::Configuration & config)
    {
      return {config("baseName"), config("X_0_s"), config("X_b0_s"), config("baseType")};
    }

    static mc_rtc::Configuration save(const mc_rbdyn::Base & b)
    {
      mc_rtc::Configuration config;
      config.add("baseName", b.baseName);
      config.add("X_0_s", b.X_0_s);
      config.add("X_b0_s", b.X_b0_s);
      config.add("baseType", b.baseType);
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<mc_rbdyn::BodySensor>
  {
    static mc_rbdyn::BodySensor load(const mc_rtc::Configuration & config)
    {
      return mc_rbdyn::BodySensor(config("name"), config("parentBody"), config("X_b_s"));
    }

    static mc_rtc::Configuration save(const mc_rbdyn::BodySensor & bs)
    {
      mc_rtc::Configuration config;
      config.add("name", bs.name());
      config.add("parentBody", bs.parentBody());
      config.add("X_b_s", bs.X_b_s());
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<mc_rbdyn::Collision>
  {
    static mc_rbdyn::Collision load(const mc_rtc::Configuration & config)
    {
      return mc_rbdyn::Collision(config("body1"), config("body2"), config("iDist"), config("sDist"), config("damping"));
    }

    static mc_rtc::Configuration save(const mc_rbdyn::Collision & c)
    {
      mc_rtc::Configuration config;
      config.add("body1", c.body1);
      config.add("body2", c.body2);
      config.add("iDist", c.iDist);
      config.add("sDist", c.sDist);
      config.add("damping", c.damping);
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>
  {
    static std::shared_ptr<mc_rbdyn::Surface> load(const mc_rtc::Configuration & config)
    {
      std::string type = config("type");
      if(type == "planar")
      {
        return std::make_shared<mc_rbdyn::PlanarSurface>(config("name"), config("bodyName"), config("X_b_s"), config("materialName"), config("planarPoints"));
      }
      else if(type == "cylindrical")
      {
        return std::make_shared<mc_rbdyn::CylindricalSurface>(config("name"), config("bodyName"), config("X_b_s"), config("materialName"), config("radius"), config("width"));
      }
      else if(type == "gripper")
      {
        return std::make_shared<mc_rbdyn::GripperSurface>(config("name"), config("bodyName"), config("X_b_s"), config("materialName"), config("pointsFromOrigin"), config("X_b_motor"), config("motorMaxTorque"));
      }
      LOG_ERROR("Unknown surface type stored " << type)
      throw(std::runtime_error("Invalid surface type stored"));
    }

    static mc_rtc::Configuration save(const std::shared_ptr<mc_rbdyn::Surface> & s)
    {
      mc_rtc::Configuration config;
      config.add("type", s->type());
      config.add("name", s->name());
      config.add("bodyName", s->bodyName());
      config.add("X_b_s", s->X_b_s());
      config.add("materialName", s->materialName());
      if(s->type() == "planar")
      {
        auto ps = static_cast<mc_rbdyn::PlanarSurface*>(s.get());
        config.add("planarPoints", ps->planarPoints());
      }
      else if(s->type() == "cylindrical")
      {
        auto cs = static_cast<mc_rbdyn::CylindricalSurface*>(s.get());
        config.add("radius", cs->radius());
        config.add("width", cs->width());
      }
      else if(s->type() == "gripper")
      {
        auto gs = static_cast<mc_rbdyn::GripperSurface*>(s.get());
        config.add("pointsFromOrigin", gs->pointsFromOrigin());
        config.add("X_b_motor", gs->X_b_motor());
        config.add("motorMaxTorque", gs->motorMaxTorque());
      }
      else
      {
        LOG_ERROR("Cannot serialize Surface of type " << s->type())
        throw(std::runtime_error("Invalid surface type"));
      }
      return config;
    }
  };

  template<>
  struct ConfigurationLoader<std::shared_ptr<mc_rbdyn::PlanarSurface>>
  {
    static std::shared_ptr<mc_rbdyn::PlanarSurface> load(const mc_rtc::Configuration & config)
    {
      std::string type = config("type");
      if(type != "planar")
      {
        LOG_ERROR("Tried to deserialize a non-planar surface into a planar surface")
        throw(std::runtime_error("Wrong surface types"));
      }
      return std::static_pointer_cast<mc_rbdyn::PlanarSurface>(ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
    }

    static mc_rtc::Configuration save(const std::shared_ptr<mc_rbdyn::PlanarSurface> & s)
    {
      return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
    }
  };

  template<>
  struct ConfigurationLoader<std::shared_ptr<mc_rbdyn::CylindricalSurface>>
  {
    static std::shared_ptr<mc_rbdyn::CylindricalSurface> load(const mc_rtc::Configuration & config)
    {
      std::string type = config("type");
      if(type != "cylindrical")
      {
        LOG_ERROR("Tried to deserialize a non-cylindrical surface into a cylindrical surface")
        throw(std::runtime_error("Wrong surface types"));
      }
      return std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
    }

    static mc_rtc::Configuration save(const std::shared_ptr<mc_rbdyn::CylindricalSurface> & s)
    {
      return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
    }
  };

  template<>
  struct ConfigurationLoader<std::shared_ptr<mc_rbdyn::GripperSurface>>
  {
    static std::shared_ptr<mc_rbdyn::GripperSurface> load(const mc_rtc::Configuration & config)
    {
      std::string type = config("type");
      if(type != "gripper")
      {
        LOG_ERROR("Tried to deserialize a non-gripper surface into a gripper surface")
        throw(std::runtime_error("Wrong surface types"));
      }
      return std::static_pointer_cast<mc_rbdyn::GripperSurface>(ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
    }

    static mc_rtc::Configuration save(const std::shared_ptr<mc_rbdyn::GripperSurface> & s)
    {
      return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
    }
  };
}
