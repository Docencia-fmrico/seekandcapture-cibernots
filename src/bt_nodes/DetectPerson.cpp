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
#include <memory>

#include "bt_nodes/DetectPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"

namespace bt_detectPerson_node
{
using std::placeholders::_1;
using namespace std::chrono_literals;

DetectPerson::DetectPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  // se suscribe a la publicación de la tf de la percepción
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

}

BT::NodeStatus
DetectPerson::tick()
{ 
  // si existe la tf pasa al siguiente y si no busca a la persona

  geometry_msgs::msg::TransformStamped odom2person_msg;
  tf2::Stamped<tf2::Transform> odom2person;
  try {
    odom2person_msg = tf_buffer_.lookupTransform(
      "odom", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(odom2person_msg, odom2person);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  if ((node_->now() - rclcpp::Time(odom2person_msg.header.stamp)) > TF_PERSON_TIMEOUT){
    RCLCPP_WARN(node_->get_logger(), "person transform not found");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_detectPerson_node

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_detectPerson_node::DetectPerson>("DetectPerson");
}