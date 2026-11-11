// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace gazebo_plugins
{

/// Planar velocity controller that leaves vertical and tipping physics untouched.
///
/// The stock gazebo_ros_planar_move plugin overwrites all three world linear
/// velocity components, including Z. That makes gravity accumulate for only
/// one update interval before vertical velocity is reset, producing a slow
/// downward drift. This plugin constrains planar control axes only:
/// - X/Y world velocity follows /cmd_vel in the robot's yaw frame.
/// - Z world velocity remains whatever gravity and collisions produce.
/// - Roll and pitch angular velocity remain physical.
/// - Yaw angular velocity follows /cmd_vel.
class GazeboRosGravityPlanarDrive : public gazebo::ModelPlugin
{
public:
  GazeboRosGravityPlanarDrive() = default;
  ~GazeboRosGravityPlanarDrive() override
  {
    if (update_connection_) {
      update_connection_.reset();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    if (!node_) {
      gzerr << "Failed to create ROS node for gravity planar drive\n";
      return;
    }

    cmd_vel_topic_ = sdf->Get<std::string>("cmd_vel", "cmd_vel").first;
    odom_topic_ = sdf->Get<std::string>("odometry", "odom").first;
    odom_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_link").first;
    publish_odom_ = sdf->Get<bool>("publish_odom", true).first;
    publish_odom_tf_ = sdf->Get<bool>("publish_odom_tf", true).first;

    const double update_rate =
      std::max(1.0, sdf->Get<double>("update_rate", 100.0).first);
    const double publish_rate =
      std::max(1.0, sdf->Get<double>("publish_rate", 50.0).first);
    update_period_ = 1.0 / update_rate;
    publish_period_ = 1.0 / publish_rate;

    covariance_x_ = sdf->Get<double>("covariance_x", 0.001).first;
    covariance_y_ = sdf->Get<double>("covariance_y", 0.001).first;
    covariance_yaw_ = sdf->Get<double>("covariance_yaw", 0.01).first;

    base_link_ = model_->GetLink(base_frame_);
    if (!base_link_) {
      const auto links = model_->GetLinks();
      if (!links.empty()) {
        base_link_ = links.front();
        base_frame_ = base_link_->GetName();
        RCLCPP_WARN(
          node_->get_logger(),
          "robot_base_frame '%s' not found; falling back to '%s'",
          base_frame_.c_str(), base_link_->GetName().c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "model has no links");
        return;
      }
    }

    initial_pose_ = base_link_->WorldPose();

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = *msg;
      });

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10));
    if (publish_odom_tf_) {
      tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100));
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & info) {
        OnUpdate(info);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "gravity-compatible planar drive loaded: cmd_vel=%s, odom=%s, "
      "z velocity and roll/pitch remain physical",
      cmd_vel_topic_.c_str(), odom_topic_.c_str());
  }

  void Reset() override
  {
    if (base_link_) {
      initial_pose_ = base_link_->WorldPose();
    }
    last_update_time_ = 0.0;
    last_publish_time_ = 0.0;
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = geometry_msgs::msg::Twist();
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (now + 1.0e-9 < last_update_time_ + update_period_) {
      return;
    }
    last_update_time_ = now;

    geometry_msgs::msg::Twist command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }

    const auto pose = base_link_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const auto linear = model_->WorldLinearVel();
    model_->SetLinearVel({
      cos_yaw * command.linear.x - sin_yaw * command.linear.y,
      sin_yaw * command.linear.x + cos_yaw * command.linear.y,
      linear.Z()});

    const auto angular = model_->WorldAngularVel();
    model_->SetAngularVel({
      angular.X(),
      angular.Y(),
      command.angular.z});

    if (now + 1.0e-9 >= last_publish_time_ + publish_period_) {
      last_publish_time_ = now;
      PublishOdometry(pose);
    }
  }

  void PublishOdometry(const ignition::math::Pose3d & current_pose)
  {
    const auto delta = initial_pose_.Inverse() * current_pose;
    const auto linear = base_link_->WorldLinearVel();
    const auto angular = base_link_->WorldAngularVel();
    const auto body_linear =
      current_pose.Rot().RotateVectorReverse(linear);
    const auto body_angular =
      current_pose.Rot().RotateVectorReverse(angular);
    const auto sim_time = world_->SimTime();
    const rclcpp::Time stamp(
      sim_time.sec, sim_time.nsec, RCL_ROS_TIME);

    if (publish_odom_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = delta.Pos().X();
      odom.pose.pose.position.y = delta.Pos().Y();
      odom.pose.pose.position.z = delta.Pos().Z();
      odom.pose.pose.orientation.w = delta.Rot().W();
      odom.pose.pose.orientation.x = delta.Rot().X();
      odom.pose.pose.orientation.y = delta.Rot().Y();
      odom.pose.pose.orientation.z = delta.Rot().Z();
      odom.twist.twist.linear.x = body_linear.X();
      odom.twist.twist.linear.y = body_linear.Y();
      odom.twist.twist.linear.z = body_linear.Z();
      odom.twist.twist.angular.x = body_angular.X();
      odom.twist.twist.angular.y = body_angular.Y();
      odom.twist.twist.angular.z = body_angular.Z();

      odom.pose.covariance.fill(0.0);
      odom.twist.covariance.fill(0.0);
      odom.pose.covariance[0] = covariance_x_;
      odom.pose.covariance[7] = covariance_y_;
      odom.pose.covariance[35] = covariance_yaw_;
      odom.twist.covariance[0] = covariance_x_;
      odom.twist.covariance[7] = covariance_y_;
      odom.twist.covariance[35] = covariance_yaw_;
      odom_pub_->publish(odom);
    }

    if (tf_pub_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = std_msgs::msg::Header();
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = delta.Pos().X();
      transform.transform.translation.y = delta.Pos().Y();
      transform.transform.translation.z = delta.Pos().Z();
      transform.transform.rotation.w = delta.Rot().W();
      transform.transform.rotation.x = delta.Rot().X();
      transform.transform.rotation.y = delta.Rot().Y();
      transform.transform.rotation.z = delta.Rot().Z();

      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.push_back(transform);
      tf_pub_->publish(tf_message);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr base_link_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist command_;

  ignition::math::Pose3d initial_pose_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_odom_{true};
  bool publish_odom_tf_{true};
  double update_period_{0.01};
  double publish_period_{0.02};
  double last_update_time_{0.0};
  double last_publish_time_{0.0};
  double covariance_x_{0.001};
  double covariance_y_{0.001};
  double covariance_yaw_{0.01};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosGravityPlanarDrive)

}  // namespace gazebo_plugins
