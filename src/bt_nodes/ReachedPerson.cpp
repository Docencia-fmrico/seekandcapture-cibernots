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
#include <math.h>
#include <cmath>

#include "bt_nodes/ReachedPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "kobuki_ros_interfaces/msg/sound.hpp"

namespace seekandcapture_cibernots
{
using std::placeholders::_1;
using namespace std::chrono_literals;


ReachedPerson::ReachedPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  // Sound publisher
  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);

}

BT::NodeStatus
ReachedPerson::tick()
{

  geometry_msgs::msg::TransformStamped odom2person_msg;
  tf2::Stamped<tf2::Transform> odom2person;

  // Search for the tf
  try {
    odom2person_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(odom2person_msg, odom2person);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Calculate distance using pythagoras theorem
  double distance = sqrt(odom2person.getOrigin().x()*odom2person.getOrigin().x() +odom2person.getOrigin().y()*odom2person.getOrigin().y());

  // If the distance is less than 1.5, the person is reached
  if (std::abs(distance) <= 1.5) {
    // Send sound
    kobuki_ros_interfaces::msg::Sound msg;
    msg.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;
    
    sound_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace seekandcapture_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture_cibernots::ReachedPerson>("ReachedPerson");
}