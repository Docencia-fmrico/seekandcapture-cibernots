# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    darknet_vision_cmd = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(os.path.join(
                            get_package_share_directory('darknet_ros'),
                            'launch',
                            'darknet_ros.launch.py'))
                          )

    darknet_cmd = Node(package='seekandcapture_cibernots',
                        executable='darknet_detection_tf',
                        output='screen',
                        parameters=[{
                          'use_sim_time': False
                        }],
                        remappings=[
                          ('input_bbxs_detection', '/darknet_ros/bounding_boxes'),
                          ('output_detection_2d', '/detection2Darray')
                        ])

    detection2d_3d_cmd = Node(package='seekandcapture_cibernots',
                                executable='detection_2d_to_3d_depth_tf',
                                output='screen',
                                parameters=[{
                                  'use_sim_time': False
                                }],
                                remappings=[
                                  ('input_depth', '/camera/depth/image_raw'),
                                  ('input_detection_2d', '/detection2Darray'),
                                  ('camera_info', '/camera/depth/camera_info'),
                                  ('output_detection_3d', '/detection3Darray')
                                ])

    detection3d_persontf_cmd = Node(package='seekandcapture_cibernots',
                                      executable='imageperson_tf',
                                      output='screen',
                                      parameters=[{
                                        'use_sim_time': False
                                      }],
                                      remappings=[
                                        ('input_detection_3d', '/detection3Darray')
                                      ])

    ld = LaunchDescription()
    ld.add_action(darknet_vision_cmd)
    ld.add_action(darknet_cmd)
    ld.add_action(detection2d_3d_cmd)
    ld.add_action(detection3d_persontf_cmd)

    return ld
