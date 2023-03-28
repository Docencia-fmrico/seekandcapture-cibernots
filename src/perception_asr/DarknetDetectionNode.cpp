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

#include <memory>

#include "perception_asr/DarknetDetectionNode.hpp"

#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"


#include "rclcpp/rclcpp.hpp"

namespace perception_asr
{

using std::placeholders::_1;

DarknetDetectionNode::DarknetDetectionNode()
: Node("darknet_detection_node")
{
  detection_bbxs_sub_ = create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
    "input_bbxs_detection", rclcpp::SensorDataQoS().reliable(),
    std::bind(&DarknetDetectionNode::detection_callback, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "output_detection_2d", rclcpp::SensorDataQoS().reliable());
}

void
DarknetDetectionNode::detection_callback(
  const darknet_ros_msgs::msg::BoundingBoxes::ConstSharedPtr & msg)
{
  if (detection_pub_->get_subscription_count() > 0) {
    /* Array of detections, to publish all the bbx detected in the image. */
    vision_msgs::msg::Detection2DArray detection_array_msg;
    detection_array_msg.header = msg->image_header;
    
    /* All the bounding boxes detected are scanned to publish them in a topic so that 
    other nodes of ROS can analyze them if they want. */
    for (const auto & bbx : msg->bounding_boxes) {
      /* A Detection2D object is created from which the center of the bbx and its size are calculated. */

      /* We are only interested in the bbx of people */
      if ((std::strcmp(bbx.class_id.c_str(), "person") == 0) && (bbx.probability > 0.75)) {

        RCLCPP_INFO(get_logger(), "bbx:%s", bbx.class_id.c_str());


        vision_msgs::msg::Detection2D detection_msg;
        detection_msg.header = msg->image_header;

        detection_msg.bbox.center.position.x = bbx.xmin + ((bbx.xmax - bbx.xmin) / 2);
        detection_msg.bbox.center.position.y = bbx.ymin + ((bbx.ymax - bbx.ymin) / 2);
        detection_msg.bbox.size_x = bbx.xmax - bbx.xmin;
        detection_msg.bbox.size_y = bbx.ymax - bbx.ymin;

        /* A Hypothesis Object is created to store the id and its probability, to associate it
        with the Detection 2D object created previously. */
        vision_msgs::msg::ObjectHypothesisWithPose obj_msg;
        obj_msg.hypothesis.class_id = bbx.class_id; /*Name of the object of the bbx */
        obj_msg.hypothesis.score = bbx.probability;

        detection_msg.results.push_back(obj_msg);
        detection_array_msg.detections.push_back(detection_msg); /* All detections are stored in the array. */
        detection_pub_->publish(detection_array_msg);
      }
    }
  }
}

}  // namespace perception_asr
