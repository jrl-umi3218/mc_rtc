#pragma once

#include <map>
#include <string>
#include <memory>

#include <boost/filesystem.hpp>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

#include <mc_rbdyn/RobotModule.h>

namespace mc_rbdyn
{
/** This struct is intended to remove the gravity contribution on all calibrated
 * force sensors. We load inertial parameters of every following link in
 * ForceSensorsCalibrator::CalibData structures, and provide methods to remove this
 * static contribution on any combination of force sensors.
 */
struct ForceSensorsCalibrator
{

  private:
    /** Structure used to hold the calibration data. This structure is *NOT*
    * intended to be constructed directly, see @ForceSensorsCalibrator for usage.
    * It is composed of 13 parameters (real numbers) that should be loaded
    * from a calibration file. **/
    struct CalibData
    {
      public:
        /** Default constructor, always returns zero contribution */
        CalibData();

        /** Load data from a file, using a gravity vector. The file
         * should contain 13 parameters in that order: mass (1),
         * rpy for X_f_ds (3), position for X_p_vb (3), wrench
         * offset (6).
         * @return Default calibration if the file does not exist,
         * throws if the file is ill-formed. */
        CalibData(const boost::filesystem::path& filename,
            const Eigen::Vector3d& gravity);

        /** Return the gravity wrench applied on the force sensor in the sensor
         * frame, i.e. the external force $f_{ext}$ is:
         * $f_{ext} = f_{mes} - wfToSensor$.
         * @param X_0_p the transform in the world frame of the parent body of the sensor
         * @param X_p_f the transform from the parent body to the sensor itself
         */
        sva::ForceVecd wfToSensor(const sva::PTransformd& X_0_p,
                                  const sva::PTransformd& X_p_f) const;

        /** Return X_f_ds, the pure rotation transform from the position given by
         * the URDF and the one estimated by calibration
         */
        const sva::PTransformd& X_f_ds() const
        {
          return X_f_ds_;
        }

        /** Return the mass of the link generating the wrench */
        double mass() const
        {
          return mass_;
        }

      private:
        /** Mass of the link generating the wrench */
        double mass_;
        /** Constant gravity wrench applied on the force sensor
         * in the world frame: $\{0, mg \} */
        sva::ForceVecd worldForce_;
        /** Local rotation between the model force sensor and real one */
        sva::PTransformd X_f_ds_;
        /** Transform from the parent body to the virtual link CoM */
        sva::PTransformd X_p_vb_;
        /** Force/torque offset */
        sva::ForceVecd offset_;
    };

  public:
    /** Default constructor: the resulting object does not modify any wrench */
    ForceSensorsCalibrator();

    /** Construct a calibrator from a robot module: for each force sensor,
     * try to load the corresponding calibration data file and construct
     * a CalibData object from it. Else, construct a default CalibData that
     * will not modify the corresponding wrench
     * @param robot_module Use calib_dir, forceSensors */
    ForceSensorsCalibrator(std::shared_ptr<mc_rbdyn::RobotModule> robot_module);

    /** Remove gravity from every wrench in-place
     * @param wrenches Map of sensorName -> ForceVecd
     * @param robot Use bodyPosW, bodyIndexByName */
    void removeGravityFromAll(std::map<std::string, sva::ForceVecd>& wrenches,
                              const mc_rbdyn::Robot& robot) const;

    /** Return the local rotation associated to the named force sensor,
     * i.e. the error between the model forceSensor and real one
     * as in ForceSensorsCalibrator::CalibData::X_f_ds() */
    const sva::PTransformd& X_fsmodel_fsactual(const std::string& name) const
    {
      return calibrations_.at(name).X_f_ds();
    }

    /** Return the mass associated to the named force sensor */
    double mass(const std::string& name) const
    {
      return calibrations_.at(name).mass();
    }

    /** Remove gravity from named wrench in-place.
     * @see removeGravityFromAll */
    void removeGravity(sva::ForceVecd& wrench,
                       const std::string& name,
                       const mc_rbdyn::Robot& robot) const;

  private:
    std::map<std::string, mc_rbdyn::ForceSensor> sensors_;
    std::map<std::string, CalibData> calibrations_;
};

}
