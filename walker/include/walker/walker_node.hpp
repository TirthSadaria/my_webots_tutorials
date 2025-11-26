/**
 * @file walker_node.hpp
 * @brief ROS2 node for walker robot with state machine
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WALKER__WALKER_NODE_HPP_
#define WALKER__WALKER_NODE_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "walker/state_context.hpp"
#include "walker/moving_forward_state.hpp"

namespace walker {

/**
 * @class WalkerNode
 * @brief ROS2 node implementing walker robot behavior using State design pattern
 *
 * This node subscribes to laser scan data, processes it through a state machine,
 * and publishes velocity commands to control the robot.
 */
class WalkerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   */
  explicit WalkerNode();

  /**
   * @brief Destructor
   */
  ~WalkerNode() override = default;

 private:
  /**
   * @brief Callback for laser scan messages
   * @param msg Laser scan message
   */
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Timer callback for state machine execution
   */
  void timerCallback();

  /**
   * @brief Publish velocity command from state context
   */
  void publishVelocity();

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State machine
  std::unique_ptr<StateContext> state_context_;

  // Parameters
  double obstacle_threshold_;
  double forward_velocity_;
  double angular_velocity_;
  double timer_period_;
};

}  // namespace walker

#endif  // WALKER__WALKER_NODE_HPP_

