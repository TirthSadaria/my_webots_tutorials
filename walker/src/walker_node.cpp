/**
 * @file walker_node.cpp
 * @brief Implementation of WalkerNode class
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#include "walker/walker_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace walker {

WalkerNode::WalkerNode() : Node("walker_node") {
  // Declare parameters
  this->declare_parameter<double>("obstacle_threshold", 0.5);
  this->declare_parameter<double>("forward_velocity", 0.2);
  this->declare_parameter<double>("angular_velocity", 0.5);
  this->declare_parameter<double>("timer_period", 0.1);

  // Get parameters
  obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
  forward_velocity_ = this->get_parameter("forward_velocity").as_double();
  angular_velocity_ = this->get_parameter("angular_velocity").as_double();
  timer_period_ = this->get_parameter("timer_period").as_double();

  // Create subscribers
  laser_scan_subscriber_ = this->create_subscription<
      sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&WalkerNode::laserScanCallback, this,
                std::placeholders::_1));

  // Create publishers with BestEffort QoS (typical for velocity commands)
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

  // Initialize state machine with MovingForwardState
  auto initial_state = new MovingForwardState(obstacle_threshold_,
                                              forward_velocity_);
  state_context_ = std::make_unique<StateContext>(initial_state);

  // Create timer for state machine execution
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(timer_period_ * 1000)),
      std::bind(&WalkerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Walker node initialized");
  RCLCPP_INFO(this->get_logger(), "Obstacle threshold: %.2f m",
              obstacle_threshold_);
  RCLCPP_INFO(this->get_logger(), "Forward velocity: %.2f m/s",
              forward_velocity_);
  RCLCPP_INFO(this->get_logger(), "Subscribing to: /scan");
  RCLCPP_INFO(this->get_logger(), "Publishing to: /cmd_vel");
}

void WalkerNode::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  static bool first_scan = true;
  if (first_scan) {
    RCLCPP_INFO(this->get_logger(), "First laser scan received! Scan has %zu ranges",
                 msg ? msg->ranges.size() : 0);
    first_scan = false;
  }
  
  if (!msg || msg->ranges.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No laser scan data received");
    return;
  }

  // Find minimum distance (excluding invalid readings)
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& range : msg->ranges) {
    if (std::isfinite(range) && range > 0.0 && range < min_distance) {
      min_distance = range;
    }
  }

  // Update state context with minimum distance and full scan data
  if (min_distance < std::numeric_limits<double>::max()) {
    state_context_->setMinDistance(min_distance);
    // Store full laser scan data for forward path checking
    state_context_->setLaserScanData(msg->ranges, msg->angle_min, msg->angle_increment);
    // Log when obstacle is detected
    if (min_distance < obstacle_threshold_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "OBSTACLE DETECTED! Distance: %.2f m (threshold: %.2f m)",
                           min_distance, obstacle_threshold_);
    }
  } else {
    state_context_->setMinDistance(-1.0);
    state_context_->setLaserScanData(msg->ranges, msg->angle_min, msg->angle_increment);
  }
}

void WalkerNode::timerCallback() {
  if (state_context_) {
    // Get current state name before update
    const char* previous_state_name = state_context_->getCurrentStateName();
    
    // Get current state info before update
    double current_linear = state_context_->getLinearVelocity();
    double current_angular = state_context_->getAngularVelocity();
    double current_distance = state_context_->getMinDistance();
    
    // Log distance info occasionally to verify laser scan is working
    static int distance_log_count = 0;
    if (++distance_log_count % 20 == 0) {  // Every 2 seconds
      if (current_distance < 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "WARNING: No valid laser scan data! Distance: %.2f",
                             current_distance);
      } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Current min distance: %.2f m", current_distance);
      }
    }
    
    // Update state machine
    state_context_->update();
    
    // Check if state changed and log it
    const char* current_state_name = state_context_->getCurrentStateName();
    if (strcmp(previous_state_name, current_state_name) != 0) {
      RCLCPP_INFO(this->get_logger(), "=== STATE: %s ===", current_state_name);
    } else {
      // Log current state periodically (every 2 seconds) even if not changed
      static int state_log_count = 0;
      if (++state_log_count % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), "=== STATE: %s ===", current_state_name);
      }
    }
    
    // Check if velocities changed (for debugging)
    if (current_linear != state_context_->getLinearVelocity() ||
        current_angular != state_context_->getAngularVelocity()) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Velocity change: linear=%.2f->%.2f, angular=%.2f->%.2f, distance=%.2f",
                   current_linear, state_context_->getLinearVelocity(),
                   current_angular, state_context_->getAngularVelocity(),
                   current_distance);
    }
    
    // Publish velocity command
    publishVelocity();
  }
}

void WalkerNode::publishVelocity() {
  if (!cmd_vel_publisher_ || !state_context_) {
    return;
  }

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = state_context_->getLinearVelocity();
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = state_context_->getAngularVelocity();

  cmd_vel_publisher_->publish(twist_msg);
  
  // Debug output (throttled to avoid spam)
  static int count = 0;
  if (++count % 10 == 0) {  // Print every 10th call (~1 second at 0.1s period)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing cmd_vel: linear=%.2f, angular=%.2f",
                         twist_msg.linear.x, twist_msg.angular.z);
  }
}

}  // namespace walker

// Main function
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<walker::WalkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

