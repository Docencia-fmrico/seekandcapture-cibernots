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

#include <string>
#include <iostream>

#include "bt_nodes/DetectPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_detectPerson_node
{
using std::placeholders::_1;
using namespace std::chrono_literals;

DetectPerson::DetectPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // // config().blackboard->get("node", node_);

  // detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
  //   "input_detection_3d", rclcpp::SensorDataQoS(),
  //   std::bind(&PersonDetectorImprovedNode::image3D_callback, this, _1));

  // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);


  // se suscribe a la publicación de la tf de la percepción
}

// Callback tf
// void
// DetectPerson::tf_callback(tf2_msgs::msg::TFMessage::UniquePtr msg)
// {

// }

BT::NodeStatus
DetectPerson::tick()
{ 
  // si existe la tf pasa al siguiente y si no busca a la persona
  tf2_msgs::msg::TFMessage TF_msgs;

  return BT::NodeStatus::RUNNING;
}

}  // namespace bt_detectPerson_node

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_detectPerson_node::DetectPerson>("DetectPerson");
}