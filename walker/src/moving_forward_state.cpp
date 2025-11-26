/**
 * @file moving_forward_state.cpp
 * @brief Implementation of MovingForwardState class
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#include "walker/moving_forward_state.hpp"
#include "walker/state_context.hpp"
#include "walker/rotating_state.hpp"

namespace walker {

MovingForwardState::MovingForwardState(double obstacle_threshold,
                                       double forward_velocity)
    : obstacle_threshold_(obstacle_threshold),
      forward_velocity_(forward_velocity) {}

void MovingForwardState::enter(StateContext* context) {
  if (context) {
    context->setLinearVelocity(forward_velocity_);
    context->setAngularVelocity(0.0);
  }
}

State* MovingForwardState::execute(StateContext* context) {
  if (!context) {
    return nullptr;
  }

  // Set forward velocity
  context->setLinearVelocity(forward_velocity_);
  context->setAngularVelocity(0.0);

  // Check for obstacle in forward path (check forward-facing sector)
  double forward_distance = context->getForwardMinDistance(60.0);  // Check ±30 degrees
  if (forward_distance > 0.0 && forward_distance < obstacle_threshold_) {
    // Obstacle detected in forward path, transition to rotating state
    return new RotatingState(obstacle_threshold_);
  }

  // Continue moving forward
  return nullptr;
}

void MovingForwardState::exit(StateContext* context) {
  // Stop linear velocity when exiting
  if (context) {
    context->setLinearVelocity(0.0);
  }
}

const char* MovingForwardState::getName() const {
  return "MOVING FORWARD";
}

}  // namespace walker

