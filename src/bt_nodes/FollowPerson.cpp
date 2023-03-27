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

#include <string>
#include <iostream>
#include <math.h>
#include <cmath>

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
  angular_pid_(0.0, M_PI_2, 0.5, 2.0),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  angular_pid_.set_pid(0.6, 0.08, 0.32);
}

BT::NodeStatus
FollowPerson::tick()
{
  geometry_msgs::msg::TransformStamped bs_link2person_msg;
  tf2::Stamped<tf2::Transform> bs_link2person;
  try {
    bs_link2person_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(bs_link2person_msg, bs_link2person);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  
  RCLCPP_WARN(node_->get_logger(), "DISTANCIA DE LA PERSONA %f,%f,%f",
              bs_link2person.getOrigin().x(),bs_link2person.getOrigin().y(),bs_link2person.getOrigin().z());

  double distance = sqrt(pow(bs_link2person.getOrigin().x(), 2) + pow(bs_link2person.getOrigin().y(), 2));
  
  double linear_vel = linear_pid_.get_output(distance);
  
  auto err_ang = std::atan2(bs_link2person.getOrigin().y(), fabs(bs_link2person.getOrigin().x()));

  RCLCPP_INFO(node_->get_logger(), "ATAN: %f", err_ang);
  
  double angular_vel = angular_pid_.get_output(err_ang);

  RCLCPP_INFO(node_->get_logger(), "ANGULAR: %f", angular_vel);
  
  geometry_msgs::msg::Twist vel_msgs;

  vel_msgs.linear.x = linear_vel;
  vel_msgs.angular.z = angular_vel;

  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}


}  // namespace seekandcapture_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture_cibernots::FollowPerson>("FollowPerson");
}
