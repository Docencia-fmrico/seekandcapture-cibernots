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

#include "bt_nodes/FollowPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>



namespace seekandcapture_cibernots
{
using namespace std::chrono_literals;

FollowPerson::FollowPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  linear_pid_(0.1, 0.7, 0.0, 0.5),
  angular_pid_(0.05, 1.0, 0.0, 0.8),
  // angular_pid_(0.01, 0.8, 0.45, 0.8),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  
  //linear_pid_.set_pid(0.4, 0.05, 0.55);
  //angular_pid_.set_pid(0.4, 0.05, 0.55);
  
}

BT::NodeStatus
FollowPerson::tick()
{
  int side = 1;

  geometry_msgs::msg::TransformStamped odom2person_msg;
  tf2::Stamped<tf2::Transform> odom2person;
  try {
    odom2person_msg = tf_buffer_.lookupTransform(
      "odom", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(odom2person_msg, odom2person);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  
  
  double linear_vel = linear_pid_.get_output(odom2person.getOrigin().x());
  
  auto err_ang = std::atan2(odom2person.getOrigin().y(), odom2person.getOrigin().x());

  RCLCPP_INFO(node_->get_logger(), "ATAN: %f", err_ang);
  
  double angular_vel = angular_pid_.get_output(err_ang);

  RCLCPP_INFO(node_->get_logger(), "ANGULAR: %f", angular_vel);

  // std::clamp(linear_vel, 0.2, 0.6);
  // std::clamp(angular_vel, -0.8, 0.8);

  // RCLCPP_INFO(node_->get_logger(), "ANGULAR: %f", angular_vel);
  
  geometry_msgs::msg::Twist vel_msgs;
  
  if(odom2person.getOrigin().y() < 0)
  {
    side = -1;
  }

  vel_msgs.linear.x = linear_vel;
  //vel_msgs.angular.z = angular_vel;
  vel_msgs.angular.z = (angular_vel) * 6.28 * side;



  RCLCPP_INFO(node_->get_logger(), "ANGULAR PUBLICADA: %f", vel_msgs.angular.z);

  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}


}  // namespace seekandcapture_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture_cibernots::FollowPerson>("FollowPerson");
}
