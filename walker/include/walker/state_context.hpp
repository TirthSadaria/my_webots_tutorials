/**
 * @file state_context.hpp
 * @brief Context class for managing state machine and providing shared data
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WALKER__STATE_CONTEXT_HPP_
#define WALKER__STATE_CONTEXT_HPP_

#include <memory>
#include <vector>
#include "walker/state.hpp"

namespace walker {

/**
 * @class StateContext
 * @brief Manages the state machine and provides access to sensor data and control
 *
 * The StateContext maintains the current state, handles state transitions,
 * and provides methods for states to access sensor readings and publish
 * velocity commands.
 */
class StateContext {
 public:
  /**
   * @brief Constructor
   * @param initial_state The initial state for the state machine
   */
  explicit StateContext(State* initial_state);

  /**
   * @brief Destructor
   */
  ~StateContext();

  /**
   * @brief Update the state machine (called each iteration)
   */
  void update();

  /**
   * @brief Transition to a new state
   * @param new_state Pointer to the new state (ownership transferred)
   */
  void transitionTo(State* new_state);

  /**
   * @brief Get the current minimum distance reading from sensors
   * @return Minimum distance in meters, or -1.0 if no reading available
   */
  double getMinDistance() const;

  /**
   * @brief Set the minimum distance reading from sensors
   * @param distance Distance in meters
   */
  void setMinDistance(double distance);

  /**
   * @brief Get the minimum distance in the forward-facing sector
   * @param angle_range_deg Angular range in degrees to check (e.g., 60 for ±30 degrees)
   * @return Minimum distance in forward sector, or -1.0 if no valid reading
   */
  double getForwardMinDistance(double angle_range_deg = 60.0) const;

  /**
   * @brief Set the laser scan data for forward path checking
   * @param ranges Vector of range readings
   * @param angle_min Minimum angle of scan
   * @param angle_increment Angle increment between readings
   */
  void setLaserScanData(const std::vector<float>& ranges, double angle_min, double angle_increment);

  /**
   * @brief Get the current linear velocity command
   * @return Linear velocity (x component)
   */
  double getLinearVelocity() const;

  /**
   * @brief Set the linear velocity command
   * @param velocity Linear velocity (x component)
   */
  void setLinearVelocity(double velocity);

  /**
   * @brief Get the current angular velocity command
   * @return Angular velocity (z component)
   */
  double getAngularVelocity() const;

  /**
   * @brief Set the angular velocity command
   * @param velocity Angular velocity (z component)
   */
  void setAngularVelocity(double velocity);

  /**
   * @brief Toggle rotation direction (alternate between CW and CCW)
   */
  void toggleRotationDirection();

  /**
   * @brief Get current rotation direction
   * @return true if clockwise, false if counterclockwise
   */
  bool isRotatingClockwise() const;

  /**
   * @brief Get the name of the current state
   * @return State name as string, or "UNKNOWN" if no state
   */
  const char* getCurrentStateName() const;

 private:
  std::unique_ptr<State> current_state_;
  double min_distance_;
  std::vector<float> laser_ranges_;
  double laser_angle_min_;
  double laser_angle_increment_;
  double linear_velocity_;
  double angular_velocity_;
  bool rotating_clockwise_;
};

}  // namespace walker

#endif  // WALKER__STATE_CONTEXT_HPP_

