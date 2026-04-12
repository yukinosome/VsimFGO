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
#include <gtsam/base/numericalDerivative.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <Eigen/Eigen>

namespace fgo::factor::camera {

  class VSimFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  protected:
    gtsam::Vector2 ll_;
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

      // 仅使用经纬度，不使用高度
      ll_ = (gtsam::Vector2() << lat_rad, lon_rad).finished();
      factorTypeID_ = FactorTypeID::VSim;
      factorName_ = "VSimFactor";
    }

    virtual ~VSimFactor() = default;

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const {
      if (H) {
        *H = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
          boost::bind(&This::evaluateError_, this, boost::placeholders::_1),
          pose, 1e-5);
      }
      return evaluateError_(pose);
    }

    [[nodiscard]] gtsam::Vector2 evaluateError_(const gtsam::Pose3 &pose) const {
      const gtsam::Point3 pose_in_gnss = pose.translation() + pose.rotation().matrix() * lb_;
      const gtsam::Point3 llh = fgo::utils::xyz2llh(pose_in_gnss);
      return (gtsam::Vector2() << llh.x() - ll_(0), llh.y() - ll_(1)).finished();
    }

    void print(const std::string &s = "VSimFactor") const {
      std::cout << s << std::endl;
      std::cout << "  measured ll(rad): " << ll_.transpose() << std::endl;
      Base::print();
    }

    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This *>(&expected);
      return e && Base::equals(*e) && gtsam::equal(ll_, e->ll_, tol);
    }

    gtsam::Vector2 get_latlon() const {return ll_;}
  };
}
#endif // CAMERA_VSIM_FACTOR_H_