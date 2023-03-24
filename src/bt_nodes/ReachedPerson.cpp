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

#include "bt_nodes/ReachedPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_ReachedPerson_node
{
using std::placeholders::_1;
using namespace std::chrono_literals;


// Decidir como va a ser la decisión de haber llegado
// ¿por tiempo? ¿por evento(button)?
ReachedPerson::ReachedPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // en caso de evento(ej:button) suscribirse al boton

}

BT::NodeStatus
ReachedPerson::tick()
{
  // si está a un metro aprox
  // y se pulsa el botón buscar a otra persona
  return BT::NodeStatus::RUNNING;
}

}  // namespace bt_ReachedPerson_node

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_ReachedPerson_node::ReachedPerson>("ReachedPerson");
}