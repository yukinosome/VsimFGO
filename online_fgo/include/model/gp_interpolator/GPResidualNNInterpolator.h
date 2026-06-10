//  GP residual neural interpolator.
//
//  First-version behavior:
//  - Forward pass uses a base GP interpolator plus an MLP residual.
//  - Returned interpolation Jacobians are delegated to the base GP only. The
//    neural residual is intentionally treated as a fixed correction at the
//    current linearization point.

#ifndef ONLINE_FGO_GPRESIDUALNNINTERPOLATOR_H
#define ONLINE_FGO_GPRESIDUALNNINTERPOLATOR_H

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "GPInterpolatorBase.h"
#include "model/nn/SimpleMLP.h"

namespace fgo::models {

  class GPResidualNNInterpolator : public GPInterpolator {
  public:
    struct Config {
      // Features: Log(T_i^-1 T_j), vel_i, omega_i, vel_j, omega_j, tau/dt, dt.
      int input_dim{20};

      // Output convention:
      //   first 6 values: pose residual xi, applied as T_gp * Exp(xi)
      //   next 6 values: velocity residual, added to GP velocity
      int output_dim{12};

      gtsam::Vector6 pose_residual_scale{gtsam::Vector6::Ones()};
      gtsam::Vector6 velocity_residual_scale{gtsam::Vector6::Ones()};
    };

    GPResidualNNInterpolator() = default;

    explicit GPResidualNNInterpolator(std::shared_ptr<GPInterpolator> base_interpolator)
      : GPResidualNNInterpolator(std::move(base_interpolator), Config{}) {}

    GPResidualNNInterpolator(std::shared_ptr<GPInterpolator> base_interpolator,
                             Config config)
      : GPInterpolator(requireBase(base_interpolator)->getQc(),
                       requireBase(base_interpolator)->getDeltat(),
                       requireBase(base_interpolator)->getTau(),
                       false,
                       false),
        base_(std::move(base_interpolator)),
        config_(std::move(config)) {}

    GPResidualNNInterpolator(std::shared_ptr<GPInterpolator> base_interpolator,
                             const std::string &weight_file)
      : GPResidualNNInterpolator(std::move(base_interpolator), weight_file, Config{}) {}

    GPResidualNNInterpolator(std::shared_ptr<GPInterpolator> base_interpolator,
                             const std::string &weight_file,
                             Config config)
      : GPResidualNNInterpolator(std::move(base_interpolator), std::move(config)) {
      loadNetwork(weight_file);
    }

    ~GPResidualNNInterpolator() override = default;

    void loadNetwork(const std::string &weight_file) {
      network_.loadFromTextFile(weight_file);
      if (network_.inputDim() != config_.input_dim) {
        throw std::runtime_error("GPResidualNNInterpolator network input dimension mismatch");
      }
      if (network_.outputDim() != config_.output_dim) {
        throw std::runtime_error("GPResidualNNInterpolator network output dimension mismatch");
      }
    }

    [[nodiscard]] bool hasNetwork() const {
      return !network_.empty();
    }

    [[nodiscard]] double getTau() const override {
      return base_ ? base_->getTau() : tau_;
    }

    void recalculate(const double &delta_t, const double &tau,
                     const gtsam::Vector6 &accI = gtsam::Vector6(),
                     const gtsam::Vector6 &accJ = gtsam::Vector6()) override {
      requireBase();
      base_->recalculate(delta_t, tau, accI, accJ);
      update(delta_t, tau);
      Qc_ = base_->getQc();
      Ad_ = base_->getAd();
    }

    void recalculate(const double &delta_t, const double &tau, const gtsam::Matrix66 &Ad,
                     const gtsam::Vector6 &accI = gtsam::Vector6(),
                     const gtsam::Vector6 &accJ = gtsam::Vector6()) override {
      requireBase();
      base_->recalculate(delta_t, tau, Ad, accI, accJ);
      update(delta_t, tau, Ad);
      Qc_ = base_->getQc();
    }

    [[nodiscard]] gtsam::Pose3 interpolatePose(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
      requireBase();
      const auto pose_gp = base_->interpolatePose(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b,
                                                  H1, H2, H3, H4, H5, H6);
      return applyPoseResidual(pose_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Pose3 interpolatePose_(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b) const override {
      requireBase();
      const auto pose_gp = base_->interpolatePose_(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b);
      return applyPoseResidual(pose_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Pose3 interpolatePose(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none,
      boost::optional<gtsam::Matrix &> H7 = boost::none,
      boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
      requireBase();
      const auto pose_gp = base_->interpolatePose(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2,
                                                  H1, H2, H3, H4, H5, H6, H7, H8);
      return applyPoseResidual(pose_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Pose3 interpolatePose_(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2) const override {
      requireBase();
      const auto pose_gp = base_->interpolatePose_(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2);
      return applyPoseResidual(pose_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Vector6 interpolateVelocity(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
      requireBase();
      const auto vel_gp = base_->interpolateVelocity(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b,
                                                     H1, H2, H3, H4, H5, H6);
      return applyVelocityResidual(vel_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Vector6 interpolateVelocity_(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n,
      const gtsam::Vector3 &omega2_b) const override {
      requireBase();
      const auto vel_gp = base_->interpolateVelocity_(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b);
      return applyVelocityResidual(vel_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Vector6 interpolateVelocity(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none,
      boost::optional<gtsam::Matrix &> H7 = boost::none,
      boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
      requireBase();
      const auto vel_gp = base_->interpolateVelocity(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2,
                                                     H1, H2, H3, H4, H5, H6, H7, H8);
      return applyVelocityResidual(vel_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Vector6 interpolateVelocity_(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2) const override {
      requireBase();
      const auto vel_gp = base_->interpolateVelocity_(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2);
      return applyVelocityResidual(vel_gp, inferResidual(buildFeatures(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b)));
    }

    [[nodiscard]] gtsam::Vector6 interpolateAcceleration(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none,
      boost::optional<gtsam::Matrix &> H7 = boost::none,
      boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
      requireBase();
      return base_->interpolateAcceleration(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2,
                                            H1, H2, H3, H4, H5, H6, H7, H8);
    }

    [[nodiscard]] gtsam::Vector6 interpolateAcceleration_(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
      const gtsam::Vector6 &acc2) const override {
      requireBase();
      return base_->interpolateAcceleration_(pose1, v1_n, omega1_b, acc1, pose2, v2_n, omega2_b, acc2);
    }

    void print(const std::string &s = "GPResidualNNInterpolator") const override {
      std::cout << s << " base=" << (base_ ? "set" : "null")
                << " network=" << (hasNetwork() ? "loaded" : "empty")
                << " input_dim=" << config_.input_dim
                << " output_dim=" << config_.output_dim << std::endl;
    }

  private:
    std::shared_ptr<GPInterpolator> base_;
    nn::SimpleMLP network_;
    Config config_;

    static GPInterpolator *requireBase(const std::shared_ptr<GPInterpolator> &base) {
      if (!base) {
        throw std::runtime_error("GPResidualNNInterpolator requires a base GP interpolator");
      }
      return base.get();
    }

    void requireBase() const {
      if (!base_) {
        throw std::runtime_error("GPResidualNNInterpolator base GP interpolator is null");
      }
    }

    [[nodiscard]] Eigen::VectorXd buildFeatures(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b) const {
      Eigen::VectorXd features = Eigen::VectorXd::Zero(config_.input_dim);
      if (config_.input_dim < 20) {
        throw std::runtime_error("GPResidualNNInterpolator requires at least 20 input features");
      }

      const gtsam::Vector6 relative_pose = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));
      features.segment<6>(0) = relative_pose;
      features.segment<3>(6) = v1_n;
      features.segment<3>(9) = omega1_b;
      features.segment<3>(12) = v2_n;
      features.segment<3>(15) = omega2_b;
      features(18) = delta_t_ > 1e-9 ? tau_ / delta_t_ : 0.0;
      features(19) = delta_t_;
      return features;
    }

    [[nodiscard]] Eigen::VectorXd inferResidual(const Eigen::VectorXd &features) const {
      if (network_.empty()) {
        return Eigen::VectorXd::Zero(config_.output_dim);
      }
      return network_.forward(features);
    }

    [[nodiscard]] gtsam::Pose3 applyPoseResidual(const gtsam::Pose3 &pose_gp,
                                                 const Eigen::VectorXd &residual) const {
      if (residual.size() < 6) {
        return pose_gp;
      }
      gtsam::Vector6 pose_residual = residual.head<6>().cwiseProduct(config_.pose_residual_scale);
      return pose_gp.compose(gtsam::Pose3::Expmap(pose_residual));
    }

    [[nodiscard]] gtsam::Vector6 applyVelocityResidual(const gtsam::Vector6 &velocity_gp,
                                                       const Eigen::VectorXd &residual) const {
      if (residual.size() < 12) {
        return velocity_gp;
      }
      gtsam::Vector6 velocity_residual = residual.segment<6>(6).cwiseProduct(config_.velocity_residual_scale);
      return velocity_gp + velocity_residual;
    }
  };

} // namespace fgo::models

#endif // ONLINE_FGO_GPRESIDUALNNINTERPOLATOR_H
