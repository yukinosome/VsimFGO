//  Copyright 2026 Institute of Automatic Control RWTH Aachen University
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

#include "integrator/VSimIntegrator.h"
#include <std_msgs/msg/bool.hpp>

namespace fgo::integrator {
  void VSimIntegrator::initialize(rclcpp::Node &node, graph::GraphBase &graphPtr, const std::string &integratorName,
                                  bool isPrimarySensor) {
    IntegratorBase::initialize(node, graphPtr, integratorName, isPrimarySensor);
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "--------------------- " << integratorName
                                                << ": start initialization... ---------------------");

    paramPtr_ = std::make_shared<IntegratorGNSSLCParams>(integratorBaseParamPtr_);
    vsimBuffer_.resize_buffer(100);

    RosParameter<double> posVarScale("GNSSFGO." + integratorName_ + ".posVarScale", 1., node);
    paramPtr_->posVarScale = posVarScale.value();

    RosParameter<std::string> noiseModelPosition("GNSSFGO." + integratorName_ + ".noiseModelPosition", "gaussian",
                                                 *rosNodePtr_);
    auto noiseModelPositionStr = noiseModelPosition.value();
    setNoiseModelFromParam(noiseModelPositionStr, paramPtr_->noiseModelPosition, "VSim Position");

    RosParameter<double> robustParamPos("GNSSFGO." + integratorName_ + ".robustParamPos", 0.5, node);
    paramPtr_->robustParamPosition = robustParamPos.value();

    RosParameter<bool> onlyRTKFixed("GNSSFGO." + integratorName_ + ".onlyRTKFixed", false, node);
    paramPtr_->onlyRTKFixed = onlyRTKFixed.value();

    RosParameter<bool> useForInitialization("GNSSFGO." + integratorName_ + ".useForInitialization", false, node);
    paramPtr_->useForInitialization = useForInitialization.value();

    RosParameter<bool> useHeaderTimestamp("GNSSFGO." + integratorName_ + ".useHeaderTimestamp", true, node);
    paramPtr_->useHeaderTimestamp = useHeaderTimestamp.value();

    RosParameter<bool> notIntegrating("GNSSFGO." + integratorName_ + ".notIntegrating", false, node);
    paramPtr_->notIntegrating = notIntegrating.value();

    RosParameter<std::string> vsimBestposTopic("GNSSFGO." + integratorName_ + ".vsimBestposTopic",
                                               "/novatel/oem7/vsim", node);
    subVSimBestpos_ = rosNodePtr_->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      vsimBestposTopic.value(),
      rclcpp::SystemDefaultsQoS(),
      std::bind(&VSimIntegrator::onVSimBestpos, this, std::placeholders::_1));

    pubVSimFactorAdded_ = rosNodePtr_->create_publisher<std_msgs::msg::Bool>(
      "fgo/factors/vsim_added", rclcpp::SystemDefaultsQoS());

    if (paramPtr_->gpType == fgo::data::GPModelType::WNOJ) {
      interpolator_ = std::make_shared<fgo::models::GPWNOJInterpolator>(
        gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
        paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
    } else if (paramPtr_->gpType == fgo::data::GPModelType::WNOA) {
      interpolator_ = std::make_shared<fgo::models::GPWNOAInterpolator>(
        gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
        paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
    } else {
      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": NO gpType chosen. Please choose.");
    }

    RCLCPP_INFO(rosNodePtr_->get_logger(), "--------------------- VSimIntegrator initialized! ---------------------");
  }

  void VSimIntegrator::onVSimBestpos(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos) {
    static const auto transSensorFromBase = sensorCalibManager_->getTransformationFromBase(sensorName_);
    rclcpp::Time msg_timestamp;
    if (paramPtr_->useHeaderTimestamp)
      msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
    else
      msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);

    const auto [sol, state] = sensor::gnss::parseNovAtelBestpos(*bestpos, msg_timestamp,
                                                               transSensorFromBase.translation());
    vsimBuffer_.update_buffer(sol, msg_timestamp);

    if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
      graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
      RCLCPP_WARN(rosNodePtr_->get_logger(), "onVSimBestpos: graph not initialized, waiting ...");
    }
  }

  bool VSimIntegrator::addFactors(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                  const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
                                  const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                  std::vector<std::pair<rclcpp::Time, fgo::data::State>> &timePredStates,
                                  gtsam::Values &values,
                                  fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                                  gtsam::KeyVector &relatedKeys) {
    if (paramPtr_->notIntegrating) {
      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << integratorName_ + " not integrating ...");
      return true;
    }

    static const auto baseToSensorTrans = sensorCalibManager_->getTransformationFromBase(sensorName_);
    static gtsam::Key pose_key_i, vel_key_i, omega_key_i, bias_key_i,
      pose_key_j, vel_key_j, omega_key_j, bias_key_j, pose_key_sync;
    static boost::circular_buffer<fgo::data::PVASolution> restVSimMeas(100);

    auto dataSensor = vsimBuffer_.get_all_buffer_and_clean();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ << ": addFactors called, vsimBuffer size: " << dataSensor.size());

    if (!restVSimMeas.empty()) {
      dataSensor.insert(dataSensor.begin(), restVSimMeas.begin(), restVSimMeas.end());
      restVSimMeas.clear();
    }

    auto pvaIter = dataSensor.begin();
    while (pvaIter != dataSensor.end()) {
      auto corrected_time = pvaIter->timestamp.seconds() - pvaIter->delay;
      auto posVarScale = paramPtr_->posVarScale;

      if (paramPtr_->onlyRTKFixed && pvaIter->type != fgo::data::GNSSSolutionType::RTKFIX) {
        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": PVA at " << std::fixed << corrected_time <<
                                                                                    " not in RTK-Fixed. Current mode "
                                                                                    << pvaIter->type
                                                                                    << " ignoring ...");
        pvaIter++;
        continue;
      }

      if (pvaIter->type != fgo::data::GNSSSolutionType::RTKFIX) {
        switch (pvaIter->type) {
          case fgo::data::GNSSSolutionType::RTKFLOAT: {
            posVarScale *= paramPtr_->varScaleRTKFloat;
            break;
          }
          case fgo::data::GNSSSolutionType::SINGLE: {
            posVarScale *= paramPtr_->varScaleSingle;
            break;
          }
          case fgo::data::GNSSSolutionType::NO_SOLUTION: {
            posVarScale *= paramPtr_->varScaleNoSolution;
            break;
          }
          default:
            break;
        }
      }

      auto syncResult = findStateForMeasurement(currentKeyIndexTimestampMap, corrected_time, paramPtr_);
      RCLCPP_DEBUG_STREAM(rosNodePtr_->get_logger(), integratorName_ << ": findStateForMeasurement status: " << syncResult.status
                             << " corrected_time: " << corrected_time);

      if (syncResult.stateJExist()) {
        pose_key_i = X(syncResult.keyIndexI);
        vel_key_i = V(syncResult.keyIndexI);
        omega_key_i = W(syncResult.keyIndexI);
        bias_key_i = B(syncResult.keyIndexI);

        pose_key_j = X(syncResult.keyIndexJ);
        vel_key_j = V(syncResult.keyIndexJ);
        omega_key_j = W(syncResult.keyIndexJ);
        bias_key_j = B(syncResult.keyIndexJ);
      } else {
        RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": VSim: NO state J found !!! ");
      }

      if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I ||
          syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J) {
        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
          pose_key_sync = pose_key_i;
        } else {
          pose_key_sync = pose_key_j;
        }

        const auto llVar = (gtsam::Vector2() << pvaIter->xyz_var(0) * posVarScale,
                            pvaIter->xyz_var(1) * posVarScale).finished();
        novatel_oem7_msgs::msg::BESTPOS bestpos_msg;
        bestpos_msg.lat = pvaIter->llh(0) * fgo::constants::rad2deg;
        bestpos_msg.lon = pvaIter->llh(1) * fgo::constants::rad2deg;
        bestpos_msg.hgt = pvaIter->llh(2);
        this->addVSimLatLonFactor(pose_key_sync,
                                  bestpos_msg,
                                  llVar,
                                  baseToSensorTrans.translation(),
                                  paramPtr_->noiseModelPosition,
                                  paramPtr_->robustParamPosition);
        if (pubVSimFactorAdded_) {
          std_msgs::msg::Bool bmsg;
          bmsg.data = true;
          pubVSimFactorAdded_->publish(bmsg);
        }
      } else if (syncResult.status == StateMeasSyncStatus::INTERPOLATED && paramPtr_->addGPInterpolatedFactor) {
        const double delta_t = syncResult.timestampJ - syncResult.timestampI;
        const double taui = syncResult.durationFromStateI;

        if (paramPtr_->gpType == fgo::data::GPModelType::WNOJ) {
          const auto [foundI, accI, fountJ, accJ] = findAccelerationToState(syncResult.keyIndexI, stateIDAccMap);
          interpolator_->recalculate(delta_t, taui, accI, accJ);
        } else {
          interpolator_->recalculate(delta_t, taui);
        }

        const auto llVar = (gtsam::Vector2() << pvaIter->xyz_var(0) * posVarScale,
                            pvaIter->xyz_var(1) * posVarScale).finished();
        novatel_oem7_msgs::msg::BESTPOS bestpos_msg;
        bestpos_msg.lat = pvaIter->llh(0) * fgo::constants::rad2deg;
        bestpos_msg.lon = pvaIter->llh(1) * fgo::constants::rad2deg;
        bestpos_msg.hgt = pvaIter->llh(2);
        this->addGPInterpolatedVSimLatLonFactor(pose_key_i,
                                                vel_key_i,
                                                omega_key_i,
                                                pose_key_j,
                                                vel_key_j,
                                                omega_key_j,
                                                bestpos_msg,
                                                llVar,
                                                baseToSensorTrans.translation(),
                                                interpolator_,
                                                paramPtr_->noiseModelPosition,
                                                paramPtr_->robustParamPosition);
        if (pubVSimFactorAdded_) {
          std_msgs::msg::Bool bmsg;
          bmsg.data = true;
          pubVSimFactorAdded_->publish(bmsg);
        }
      } else if (syncResult.status == StateMeasSyncStatus::CACHED) {
        restVSimMeas.push_back(*pvaIter);
      }
      pvaIter++;
    }
    return true;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(fgo::integrator::VSimIntegrator, fgo::integrator::IntegratorBase)
