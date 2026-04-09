// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef IRT_NAV_COMMON_NAVIGATION_TOOL_ROS_H
#define IRT_NAV_COMMON_NAVIGATION_TOOL_ROS_H

#pragma once

#include <gtsam/geometry/Point3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "navigation_tools.h"

namespace irt_nav_common::ros
{
    inline double calculate_distance(const geometry_msgs::msg::Vector3& p1,
                                     const geometry_msgs::msg::Vector3& p2)
    {
      auto p1_ = gtsam::Point3(p1.x, p1.y, p1.z);
      auto p2_ = gtsam::Point3(p2.x, p2.y, p2.z);
      return gtsam::distance3(p1_, p2_);
    }

    inline double calculate_distance(const gtsam::Point3& p1,
                                     const geometry_msgs::msg::Vector3& p2)
    {
      try {
        auto p2_ = gtsam::Point3(p2.x, p2.y, p2.z);
        return gtsam::distance3(p1, p2_);
      }
      catch (std::exception& ex)
      {
        return -1;
      }
    }

}
#endif //IRT_NAV_COMMON_NAVIGATION_TOOL_ROS_H
