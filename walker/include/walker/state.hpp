/**
 * @file state.hpp
 * @brief Abstract base class for State design pattern implementation
 *
 * Copyright (c) 2024 Tirth Sadaria
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WALKER__STATE_HPP_
#define WALKER__STATE_HPP_

namespace walker {

// Forward declaration
class StateContext;

/**
 * @class State
 * @brief Abstract base class for all states in the walker state machine
 *
 * Implements the State design pattern where each state encapsulates
 * behavior and transitions. States are managed by StateContext.
 */
class State {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~State() = default;

  /**
   * @brief Called when entering this state
   * @param context The state context managing state transitions
   */
  virtual void enter(StateContext* context) = 0;

  /**
   * @brief Called each iteration to execute state behavior
   * @param context The state context managing state transitions
   * @return Pointer to next state, or nullptr if no transition
   */
  virtual State* execute(StateContext* context) = 0;

  /**
   * @brief Called when exiting this state
   * @param context The state context managing state transitions
   */
  virtual void exit(StateContext* context) = 0;

  /**
   * @brief Get the name of this state for logging
   * @return State name as string
   */
  virtual const char* getName() const = 0;
};

}  // namespace walker

#endif  // WALKER__STATE_HPP_

