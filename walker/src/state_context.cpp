/**
 * @file state_context.cpp
 * @brief Implementation of StateContext class
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#include "walker/state_context.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace walker {

StateContext::StateContext(State* initial_state)
    : current_state_(initial_state),
      min_distance_(-1.0),
      laser_angle_min_(0.0),
      laser_angle_increment_(0.0),
      linear_velocity_(0.0),
      angular_velocity_(0.0),
      rotating_clockwise_(true) {
  if (current_state_) {
    current_state_->enter(this);
  }
}

StateContext::~StateContext() = default;

void StateContext::update() {
  if (!current_state_) {
    return;
  }

  State* next_state = current_state_->execute(this);
  if (next_state != nullptr && next_state != current_state_.get()) {
    transitionTo(next_state);
  }
}

void StateContext::transitionTo(State* new_state) {
  if (!new_state) {
    return;
  }

  if (current_state_) {
    current_state_->exit(this);
  }

  current_state_.reset(new_state);
  current_state_->enter(this);
}

double StateContext::getMinDistance() const { return min_distance_; }

void StateContext::setMinDistance(double distance) { min_distance_ = distance; }

void StateContext::setLaserScanData(const std::vector<float>& ranges,
                                     double angle_min,
                                     double angle_increment) {
  laser_ranges_ = ranges;
  laser_angle_min_ = angle_min;
  laser_angle_increment_ = angle_increment;
}

double StateContext::getForwardMinDistance(double angle_range_deg) const {
  if (laser_ranges_.empty() || laser_angle_increment_ == 0.0) {
    return -1.0;
  }

  // Convert angle range to radians
  double half_range_rad = (angle_range_deg * M_PI / 180.0) / 2.0;

  // Find minimum distance in forward-facing sector (centered at 0 radians)
  double min_forward_distance = std::numeric_limits<double>::max();
  bool found_valid = false;

  for (size_t i = 0; i < laser_ranges_.size(); ++i) {
    // Calculate angle for this reading
    double angle = laser_angle_min_ + (i * laser_angle_increment_);
    
    // Normalize angle to [-π, π] range
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    
    // Check if this reading is in the forward sector
    if (std::abs(angle) <= half_range_rad) {
      float range = laser_ranges_[i];
      if (std::isfinite(range) && range > 0.0 && range < min_forward_distance) {
        min_forward_distance = range;
        found_valid = true;
      }
    }
  }

  return found_valid ? min_forward_distance : -1.0;
}

double StateContext::getLinearVelocity() const { return linear_velocity_; }

void StateContext::setLinearVelocity(double velocity) {
  linear_velocity_ = velocity;
}

double StateContext::getAngularVelocity() const { return angular_velocity_; }

void StateContext::setAngularVelocity(double velocity) {
  angular_velocity_ = velocity;
}

void StateContext::toggleRotationDirection() {
  rotating_clockwise_ = !rotating_clockwise_;
}

bool StateContext::isRotatingClockwise() const { return rotating_clockwise_; }

const char* StateContext::getCurrentStateName() const {
  if (current_state_) {
    return current_state_->getName();
  }
  return "UNKNOWN";
}

}  // namespace walker

