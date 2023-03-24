// Copyright 2021 Intelligent Robotics Lab
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

#ifndef TF2_DETECTOR__PERSONDETECTORIMPROVEDNODE_HPP_
#define TF2_DETECTOR__PERSONDETECTORIMPROVEDNODE_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vision_msgs/msg/detection3_d_array.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace tf2_detector
{

class PersonDetectorImprovedNode : public rclcpp::Node
{
public:
  PersonDetectorImprovedNode();

private:
  void image3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr detection3D_msg);

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace tf2_detector
#endif  // TF2_DETECTOR__PERSONDETECTORIMPROVEDNODE_HPP_
