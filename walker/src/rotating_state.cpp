/**
 * @file rotating_state.cpp
 * @brief Implementation of RotatingState class
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#include "walker/rotating_state.hpp"
#include "walker/state_context.hpp"
#include "walker/moving_forward_state.hpp"

namespace walker {

RotatingState::RotatingState(double clear_threshold, double angular_velocity)
    : clear_threshold_(clear_threshold), angular_velocity_(angular_velocity) {}

void RotatingState::enter(StateContext* context) {
  if (context) {
    // Toggle rotation direction for alternating behavior
    context->toggleRotationDirection();
    context->setLinearVelocity(0.0);
    // Set angular velocity based on direction
    double vel = context->isRotatingClockwise() ? angular_velocity_
                                                 : -angular_velocity_;
    context->setAngularVelocity(vel);
  }
}

State* RotatingState::execute(StateContext* context) {
  if (!context) {
    return nullptr;
  }

  // Continue rotating
  context->setLinearVelocity(0.0);
  double vel = context->isRotatingClockwise() ? angular_velocity_
                                               : -angular_velocity_;
  context->setAngularVelocity(vel);

  // Check if forward path is clear (check forward-facing sector, not just minimum distance)
  // Use a larger threshold (1.0m) to ensure there's enough room to move forward
  double forward_distance = context->getForwardMinDistance(60.0);  // Check ±30 degrees
  if (forward_distance > 0.0 && forward_distance > 1.0) {
    // Forward path is clear (at least 1.0m ahead), transition to moving forward
    return new MovingForwardState(clear_threshold_, 0.2);
  }

  // Continue rotating
  return nullptr;
}

void RotatingState::exit(StateContext* context) {
  // Stop angular velocity when exiting
  if (context) {
    context->setAngularVelocity(0.0);
  }
}

const char* RotatingState::getName() const {
  return "ROTATING";
}

}  // namespace walker

