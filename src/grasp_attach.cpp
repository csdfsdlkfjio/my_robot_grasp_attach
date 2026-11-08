// Copyright 2026 robot
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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <string>

#include "my_robot_grasp_attach/grasp_attach_command.hpp"

namespace my_robot_grasp_attach
{

/// Welds a grasped model to a gripper link with a fixed joint at run time.
/**
 * Gazebo Classic applies position-controlled joints with a teleport, so the
 * finger pads generate no steady load transfer and a friction pinch grasp was
 * measured not to lift a cube (the pads slide up its faces while the cube
 * stays on its pedestal). A fixed joint created through the physics engine is
 * how the simulation is told that the grasp holds: from then on the cube is
 * part of the robot's kinematic tree and follows the hand exactly.
 *
 * Commands arrive on /grasp_attach and are applied on the simulation thread
 * (world update begin), because creating or detaching a joint from the ROS
 * executor thread while physics is stepping is a data race.
 */
class GraspAttach : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {
      gzerr << "[grasp_attach] could not create a ROS node" << std::endl;
      return;
    }

    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/grasp_attach/status", rclcpp::QoS(10));
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/grasp_attach", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = message->data;
        has_pending_ = true;
      });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) {OnWorldUpdate();});

    gzmsg << "[grasp_attach] ready on /grasp_attach: send 'attach "
             "<parent_model> <parent_link> <child_model> <child_link>' or "
             "'detach'" << std::endl;
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
    has_pending_ = false;
    DetachLocked();
  }

  ~GraspAttach() override
  {
    update_connection_.reset();
  }

private:
  /// Find a link by its bare or its model-scoped name.
  /**
   * Gazebo tools print links as "<model>::<link>", and a spawned model may
   * keep that scoped name internally, so GetLink() with the bare name is not
   * enough on its own.
   */
  static gazebo::physics::LinkPtr FindLink(
    const gazebo::physics::ModelPtr & model, const std::string & name)
  {
    const std::string scoped = model->GetName() + "::" + name;
    for (const gazebo::physics::LinkPtr & link : model->GetLinks()) {
      if (link->GetName() == name || link->GetScopedName() == scoped) {
        return link;
      }
    }
    return gazebo::physics::LinkPtr();
  }

  void OnWorldUpdate()
  {
    std::string text;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pending_) {
        return;
      }
      text = pending_;
      pending_.clear();
      has_pending_ = false;
    }

    AttachCommand command;
    std::string error;
    if (!parse_attach_command(text, command, error)) {
      PublishStatus("failed: " + error);
      return;
    }

    if (command.detach) {
      std::lock_guard<std::mutex> lock(mutex_);
      DetachLocked();
      PublishStatus("detached");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    AttachLocked(command);
  }

  /// Create the fixed joint; caller holds mutex_.
  void AttachLocked(const AttachCommand & command)
  {
    if (joint_) {
      // The task node repeats the command until it sees "attached"; a second
      // attach of the same pair must stay a no-op rather than adding joints.
      PublishStatus("attached");
      return;
    }

    gazebo::physics::ModelPtr parent_model =
      world_->ModelByName(command.parent_model);
    gazebo::physics::ModelPtr child_model =
      world_->ModelByName(command.child_model);
    if (!parent_model || !child_model) {
      PublishStatus(
        "failed: model '" + command.parent_model + "' or '" +
        command.child_model + "' does not exist");
      return;
    }

    gazebo::physics::LinkPtr parent_link =
      FindLink(parent_model, command.parent_link);
    if (!parent_link) {
      PublishStatus(
        "failed: model '" + command.parent_model + "' has no link '" +
        command.parent_link + "'");
      return;
    }
    gazebo::physics::LinkPtr child_link =
      FindLink(child_model, command.child_link);
    if (!child_link) {
      PublishStatus(
        "failed: model '" + command.child_model + "' has no link '" +
        command.child_link + "'");
      return;
    }

    joint_ = world_->Physics()->CreateJoint("fixed", parent_model);
    if (!joint_) {
      PublishStatus("failed: the physics engine refused to create a joint");
      return;
    }
    joint_->SetName("grasp_attach_joint");
    joint_->SetModel(parent_model);
    joint_->Attach(parent_link, child_link);
    joint_->Init();

    gzmsg << "[grasp_attach] attached " << command.child_model << "::" <<
      command.child_link << " to " << command.parent_model << "::" <<
      command.parent_link << std::endl;
    PublishStatus("attached");
  }

  /// Break the joint; caller holds mutex_.
  void DetachLocked()
  {
    if (!joint_) {
      return;
    }
    // Gazebo Classic has no PhysicsEngine::RemoveJoint, so the joint stays in
    // the model's list. Detach() empties its parent and child links, which
    // makes it inert; a later attach creates a new joint.
    joint_->Detach();
    joint_.reset();
    gzmsg << "[grasp_attach] detached" << std::endl;
  }

  void PublishStatus(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  std::mutex mutex_;
  std::string pending_;
  bool has_pending_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(GraspAttach)

}  // namespace my_robot_grasp_attach
