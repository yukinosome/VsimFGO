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

#ifndef ONLINE_FGO_INTEGRATOR_VSIM_H
#define ONLINE_FGO_INTEGRATOR_VSIM_H
#pragma once

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <std_msgs/msg/bool.hpp>

#include "IntegratorBase.h"
#include "sensor/gnss/GNSSDataParser.h"

namespace fgo::integrator {
  class VSimIntegrator : public IntegratorBase {
    IntegratorGNSSLCParamsPtr paramPtr_;
    std::shared_ptr<fgo::models::GPInterpolator> interpolator_;

    fgo::data::CircularDataBuffer<fgo::data::PVASolution> vsimBuffer_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subVSimBestpos_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubVSimFactorAdded_;

  public:
    explicit VSimIntegrator() = default;
    ~VSimIntegrator() override = default;

    void initialize(rclcpp::Node &node, fgo::graph::GraphBase &graphPtr, const std::string &integratorName,
                    bool isPrimarySensor = false) override;

    bool addFactors(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                    const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
                    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                    std::vector<std::pair<rclcpp::Time, fgo::data::State>> &timePredStates,
                    gtsam::Values &values,
                    fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                    gtsam::KeyVector &relatedKeys) override;

    bool fetchResult(const gtsam::Values &result,
                     const gtsam::Marginals &martinals,
                     const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                     fgo::data::State &optState) override {
      return true;
    }

    void dropMeasurementBefore(double timestamp) override {
      vsimBuffer_.cleanBeforeTime(timestamp);
    }

    bool checkHasMeasurements() override {
      return vsimBuffer_.size() != 0;
    }

    void cleanBuffers() override {
      const auto buffer = vsimBuffer_.get_all_buffer_and_clean();
    }

  private:
    void onVSimBestpos(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos);
  };
}

#endif // ONLINE_FGO_INTEGRATOR_VSIM_H
