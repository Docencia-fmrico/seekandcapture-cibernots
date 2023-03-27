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

    image2tf_person_cmd = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(os.path.join(
                            get_package_share_directory('seekandcapture_cibernots'),
                            'launch',
                            'image2tf_person.launch.py'))
                          )
    
    seekandcapture_cmd = Node(package='seekandcapture_cibernots',
                                  executable='seekandcapture',
                                  output='screen',
                                  parameters=[{
                                    'use_sim_time': False
                                  }],
                                  remappings=[
                                    ('output_vel', '/cmd_vel'),
                                    ('output_sound', '/commands/sound')
                                  ])

    ld = LaunchDescription()
    ld.add_action(seekandcapture_cmd)
    ld.add_action(image2tf_person_cmd)

    return ld
