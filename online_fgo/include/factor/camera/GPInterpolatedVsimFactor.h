//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//


#ifndef ONLINE_FGO_GPINTERPOLATEDVSIMFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDVSIMFACTOR_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeID.h"
#include "factor/FactorType.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>

namespace fgo::factor {
  class GPInterpolatedVSimFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  private:
    gtsam::Point3 pos_;
    gtsam::Vector3 lb_;
    double tau_{};
    bool useAutoDiff_ = false;

    typedef GPInterpolatedVSimFactor This;
    typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

    // interpolator
    GPBase GPbase_;

  public:
    GPInterpolatedVSimFactor() = default; /* Default constructor */

    /**
     * Constructor
     * @param body_P_sensor transformation from body to sensor
     */
    GPInterpolatedVSimFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key omega_i,
                           gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key omega_j,
                           const novatel_oem7_msgs::msg::BESTPOS &vsimMsg,
                           const gtsam::Vector3 &lb,
                           const gtsam::SharedNoiseModel &model,
                           const std::shared_ptr<fgo::models::GPInterpolator> &interpolator, bool useAutoDiff = false) :
      Base(model, pose_i, vel_i, omega_i, pose_j, vel_j, omega_j),
      lb_(lb), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPbase_(interpolator) {
      // Convert BESTPOS message latitude, longitude, height to ECEF coordinates
      double lat_rad = vsimMsg.lat * M_PI / 180.0;
      double lon_rad = vsimMsg.lon * M_PI / 180.0;
      double hgt_m = vsimMsg.hgt;
      
      // Convert geodetic coordinates (lat, lon, height) to ECEF
      gtsam::Point3 llh_point(lat_rad, lon_rad, hgt_m);
      pos_ = fgo::utils::llh2xyz(llh_point);
      
      factorTypeID_ = FactorTypeID::GPVSim;
      factorName_ = "GPInterpolatedVSimFactor";
    }

    virtual ~GPInterpolatedVSimFactor() = default;

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector
    evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i,
                  const gtsam::Pose3 &pose_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                  boost::optional<gtsam::Matrix &> H6 = boost::none) const {
      if (useAutoDiff_) {
        if (H1)
          *H1 = gtsam::numericalDerivative61<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
        if (H2)
          *H2 = gtsam::numericalDerivative62<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative63<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
        if (H4)
          *H4 = gtsam::numericalDerivative64<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
        if (H5)
          *H5 = gtsam::numericalDerivative65<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
        if (H6)
          *H6 = gtsam::numericalDerivative66<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
      } else {
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hpose, Hrot, Hrot2;

        gtsam::Pose3 pose_interp;

        if (H1 || H2 || H3 || H4 || H5 || H6) {
          pose_interp = GPbase_->interpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j,
                                          Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
        } else {
          pose_interp = GPbase_->interpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
        }

        const auto ePos = pose_interp.translation(Hpose) + pose_interp.rotation(Hrot).rotate(lb_, Hrot2) - pos_;

        if (H1) *H1 = (Hpose + Hrot2 * Hrot) * Hint1_P;
        if (H2) *H2 = (Hpose + Hrot2 * Hrot) * Hint2_P;
        if (H3) *H3 = (Hpose + Hrot2 * Hrot) * Hint3_P;
        if (H4) *H4 = (Hpose + Hrot2 * Hrot) * Hint4_P;
        if (H5) *H5 = (Hpose + Hrot2 * Hrot) * Hint5_P;
        if (H6) *H6 = (Hpose + Hrot2 * Hrot) * Hint6_P;

        return ePos;
      }
      return pos_ - (pose_i.translation() + pose_i.rotation().matrix() * lb_);  // Placeholder return
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &pose_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i,
                   const gtsam::Pose3 &pose_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j) const {
      const auto pose = GPbase_->interpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
      return pose.translation() + pose.rotation().rotate(lb_) - pos_;
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) override {
      const auto pose_i = values.at<gtsam::Pose3>(key1());
      const auto vel_i = values.at<gtsam::Vector3>(key2());
      const auto omega_i = values.at<gtsam::Vector3>(key3());
      const auto pose_j = values.at<gtsam::Pose3>(key4());
      const auto vel_j = values.at<gtsam::Vector3>(key5());
      const auto omega_j = values.at<gtsam::Vector3>(key6());
      const auto liftedStates = (gtsam::Vector(24) << pose_i.rotation().rpy(),
        pose_i.translation(),
        vel_i, omega_i,
        pose_j.rotation().rpy(),
        pose_j.translation(),
        vel_j, omega_j).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) override {
      assert(state.size() != 24);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Vector3(state.block<3, 1>(6, 0)));
        values.insert(key3(), gtsam::Vector3(state.block<3, 1>(9, 0)));
        values.insert(key4(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(12, 0)),
                                           gtsam::Point3(state.block<3, 1>(15, 0))));
        values.insert(key5(), gtsam::Vector3(state.block<3, 1>(18, 0)));
        values.insert(key6(), gtsam::Vector3(state.block<3, 1>(21, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << getName() << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] gtsam::Point3 measured() const {
      return pos_;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Point3() << this->pos_).finished(),
                                          (gtsam::Point3() << e->pos_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterpolatedVSimFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedVSimFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(pos_);
    }

  };
}

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedVSimFactor> : public Testable<fgo::factor::GPInterpolatedVSimFactor> {
  };
}

#endif //ONLINE_FGO_GPINTERPOLATEDVSIMFACTOR_H