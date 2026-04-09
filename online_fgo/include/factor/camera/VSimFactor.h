#ifndef CAMERA_VSIM_FACTOR_H_
#define CAMERA_VSIM_FACTOR_H_

#include "factor/FactorType.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeID.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <Eigen/Eigen>

namespace fgo::factor::camera {

  class VSimFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  protected:
    gtsam::Point3 pos_;
    gtsam::Vector3 lb_;

    typedef VSimFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
    bool useAutoDiff_ = false;

  public:
    VSimFactor() = default;

    VSimFactor(const gtsam::Key &poseKey,
               const novatel_oem7_msgs::msg::BESTPOS &vsimMsg,
               const gtsam::Vector3 &lb,
               const gtsam::SharedNoiseModel &model,
               bool useAutoDiff = false) : Base(model, poseKey),
                                           lb_(lb), useAutoDiff_(useAutoDiff) {
      // 将BESTPOS消息中的纬度、经度、高度转换为ECEF坐标
      double lat_rad = vsimMsg.lat * M_PI / 180.0;
      double lon_rad = vsimMsg.lon * M_PI / 180.0;
      double hgt_m = vsimMsg.hgt;
      
      // 将大地坐标(纬度、经度、高度)转换为ECEF
      gtsam::Point3 llh_point(lat_rad, lon_rad, hgt_m);
      pos_ = fgo::utils::llh2xyz(llh_point);
      factorTypeID_ = FactorTypeID::VSim;
      factorName_ = "VSimFactor";
    }

    virtual ~VSimFactor() = default;

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const {
      if (useAutoDiff_) {
        return Base::evaluateError(pose, H);
      }

      gtsam::Point3 pose_in_gnss = pose.translation() + pose.rotation().matrix() * lb_;
      gtsam::Vector3 error = pose_in_gnss - pos_;

      if (H) {
        H->resize(3, 6);
        H->setZero();
        H->block(0, 0, 3, 3) = gtsam::skewSymmetric(pose.rotation().matrix() * lb_);
        H->block(0, 3, 3, 3) = gtsam::Matrix3::Identity();
      }
      return error;
    }

    void print(const std::string &s = "VSimFactor") const {
      std::cout << s << std::endl;
      std::cout << "  measured pos: " << pos_.transpose() << std::endl;
      Base::print();
    }

    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This *>(&expected);
      return e && Base::equals(*e) && gtsam::equal(pos_, e->pos_, tol);
    }

    gtsam::Point3 get_position() const {return pos_;}
  };
}
#endif // CAMERA_VSIM_FACTOR_H_