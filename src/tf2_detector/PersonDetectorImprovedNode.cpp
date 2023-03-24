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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "tf2_detector/PersonDetectorImprovedNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace tf2_detector
{

using std::placeholders::_1;
using namespace std::chrono_literals;

PersonDetectorImprovedNode::PersonDetectorImprovedNode()
: Node("person_detector_improved"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
    detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "input_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&PersonDetectorImprovedNode::image3D_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

}

void
PersonDetectorImprovedNode::image3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr detection3D_msg)
{
  // Publica una transformada en la posicion de cada persona detectada mediante bounding boxes
  for (const auto & person : detection3D_msg->detections) {
    tf2::Transform camera2person;
    /*En z, se detecta la distancia a la que se encuentra el robot de la persona, es decir, si te acercas disminuye, si te alejas aumenta.*/
    camera2person.setOrigin(tf2::Vector3(person.bbox.center.position.x, person.bbox.center.position.y, person.bbox.center.position.z));
    camera2person.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    RCLCPP_INFO(get_logger(), "FRAME ID DEL MENSAJE: %s", detection3D_msg->header.frame_id.c_str());
    geometry_msgs::msg::TransformStamped odom2camera_msg;
    tf2::Stamped<tf2::Transform> odom2camera;
    try {
      odom2camera_msg = tf_buffer_.lookupTransform(
        "odom", detection3D_msg->header.frame_id.c_str(),
        tf2::timeFromSec(rclcpp::Time(detection3D_msg->header.stamp).seconds() - 0.3));
      tf2::fromMsg(odom2camera_msg, odom2camera);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Camera transform not found: %s", ex.what());
      return;
    }

    tf2::Transform odom2person = odom2camera * camera2person;

    geometry_msgs::msg::TransformStamped odom2person_msg;
    odom2person_msg.transform = tf2::toMsg(odom2person);

    odom2person_msg.header.stamp = detection3D_msg->header.stamp;
    odom2person_msg.header.frame_id = "odom";
    odom2person_msg.child_frame_id = "detected_person";
    RCLCPP_INFO(get_logger(), "TRANSFORMADA PERSONA PUBLICADA");
    tf_broadcaster_->sendTransform(odom2person_msg);
  }
}

}  // namespace tf2_detector
