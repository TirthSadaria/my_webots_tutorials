/**
 * @file moving_forward_state.hpp
 * @brief State for moving the robot forward
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WALKER__MOVING_FORWARD_STATE_HPP_
#define WALKER__MOVING_FORWARD_STATE_HPP_

#include "walker/state.hpp"

namespace walker {

class StateContext;

/**
 * @class MovingForwardState
 * @brief State that moves the robot forward until an obstacle is detected
 *
 * This state publishes a forward linear velocity. When an obstacle is
 * detected (distance below threshold), it transitions to RotatingState.
 */
class MovingForwardState : public State {
 public:
  /**
   * @brief Constructor
   * @param obstacle_threshold Distance threshold in meters for obstacle detection
   * @param forward_velocity Forward linear velocity in m/s
   */
  MovingForwardState(double obstacle_threshold, double forward_velocity);

  /**
   * @brief Destructor
   */
  ~MovingForwardState() override = default;

  /**
   * @brief Called when entering this state
   * @param context The state context
   */
  void enter(StateContext* context) override;

  /**
   * @brief Execute state behavior and check for transitions
   * @param context The state context
   * @return Pointer to RotatingState if obstacle detected, nullptr otherwise
   */
  State* execute(StateContext* context) override;

  /**
   * @brief Called when exiting this state
   * @param context The state context
   */
  void exit(StateContext* context) override;

  /**
   * @brief Get the name of this state
   * @return State name
   */
  const char* getName() const override;

 private:
  double obstacle_threshold_;
  double forward_velocity_;
};

}  // namespace walker

#endif  // WALKER__MOVING_FORWARD_STATE_HPP_

