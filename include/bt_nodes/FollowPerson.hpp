// Copyright 2023 Intelligent Robotics Lab
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

#ifndef BT_NODES__FOLLOWPERSON_HPP_
#define BT_NODES__FOLLOWPERSON_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pid/PIDController.hpp"
#include <memory>
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

namespace seekandcapture_cibernots
{

class FollowPerson : public BT::ActionNodeBase
{
public:
  explicit FollowPerson(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  PIDController linear_pid_, angular_pid_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace seekandcapture_cibernots

#endif  // BT_NODES__FOLLOWPERSON_HPP_
