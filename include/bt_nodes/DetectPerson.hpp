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

#ifndef BT_DETECTPERSON_NODE__DETECTPERSON_HPP_
#define BT_DETECTPERSON_NODE__DETECTPERSON_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//#include "tf2_msgs/msg/tf_message.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_detectPerson_node
{

class DetectPerson : public BT::ActionNodeBase
{
public:
  explicit DetectPerson(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:

  // Callback tf (get the last scan)
  // void tf_callback(tf2_msgs::msg::TFMessage::UniquePtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  
//   rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
//   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

//   tf2::BufferCore tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;
};

}  // namespace bt_detectPerson_node

#endif  // BT_DETECTPERSON_NODE__DETECTPERSON_HPP_
