/**
 * @file rotating_state.hpp
 * @brief State for rotating the robot in place
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WALKER__ROTATING_STATE_HPP_
#define WALKER__ROTATING_STATE_HPP_

#include "walker/state.hpp"

namespace walker {

class StateContext;

/**
 * @class RotatingState
 * @brief State that rotates the robot in place until path is clear
 *
 * This state rotates the robot alternating between clockwise and
 * counterclockwise directions. When the path ahead is clear (distance
 * above threshold), it transitions to MovingForwardState.
 */
class RotatingState : public State {
 public:
  /**
   * @brief Constructor
   * @param clear_threshold Distance threshold in meters for clear path
   * @param angular_velocity Angular velocity in rad/s
   */
  RotatingState(double clear_threshold, double angular_velocity = 0.5);

  /**
   * @brief Destructor
   */
  ~RotatingState() override = default;

  /**
   * @brief Called when entering this state
   * @param context The state context
   */
  void enter(StateContext* context) override;

  /**
   * @brief Execute state behavior and check for transitions
   * @param context The state context
   * @return Pointer to MovingForwardState if path is clear, nullptr otherwise
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
  double clear_threshold_;
  double angular_velocity_;
};

}  // namespace walker

#endif  // WALKER__ROTATING_STATE_HPP_

