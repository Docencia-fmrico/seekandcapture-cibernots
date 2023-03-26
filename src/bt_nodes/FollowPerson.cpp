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
using std::placeholders::_1;
using namespace std::chrono_literals;

FollowPerson::FollowPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  linear_pid_(0.1, 0.7, 0.0, 0.5),
  angular_pid_(0.1, 0.8, 0.3, 0.9),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::scan_callback, this, _1));

  linear_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  ang_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // se suscribe a la publicación de la tf de la percepción
  
  linear_pid_.set_pid(0.4, 0.05, 0.55);
  angular_pid_.set_pid(0.4, 0.05, 0.55);
  
}

void
FollowPerson::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  int n = 0;
  is_obstacle_ = false;
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  for (int j = 0; j < MIN_POS; j++) {
    if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) &&
      last_scan_->ranges[j] < OBSTACLE_DISTANCE)
    {
      is_obstacle_ = true;
      object_position_[n] = last_scan_->ranges[j];
    }
    object_position_[n] = 1e9;
    n++;
  }

  for (int j = MAX_POS; j < last_scan_->ranges.size(); j++) {
    if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) &&
      last_scan_->ranges[j] < OBSTACLE_DISTANCE)
    {
      is_obstacle_ = true;
      object_position_[n] = last_scan_->ranges[j];
    }
    object_position_[n] = 1e9;
    n++;
  }
}

int
FollowPerson::obstacle_side()
{
  int n;
  int side;

  for (int j = 0; j < LEN_MEDS; j++) {
    if (object_position_[j] < object_position_[n]) {
      n = j;
    }
  }

  if (n < MIN_POS) {
    side = -1;
  } else {
    side = 1;
  }

  return side;
}

BT::NodeStatus
FollowPerson::tick()
{

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
  
  
  double linear_vel = linear_pid_.get_output(odom2person.getOrigin().z());
  
  auto err_ang = std::atan2(odom2person.getOrigin().y(), odom2person.getOrigin().z());
  
  double angular_vel = angular_pid_.get_output(err_ang);

  std::clamp(linear_vel, 0.2, 0.6);
  std::clamp(angular_vel, 0.3, 0.8);
  
  geometry_msgs::msg::Twist vel_msgs;

  if (!is_obstacle_) {
    vel_msgs.linear.x = linear_vel;
    vel_msgs.angular.z = angular_vel;
  } else {
    side_ = obstacle_side();
    vel_msgs.linear.x = linear_vel;
    vel_msgs.angular.z = SPEED_ANGULAR * side_;
  }
  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}


}  // namespace seekandcapture_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture_cibernots::FollowPerson>("FollowPerson");
}
