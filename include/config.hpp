#pragma once
// ###########################################################
// This file contains project-wide configuration settings.   #
// ###########################################################

// # --- PID --- #
// Lateral PID controller
constexpr float KP_LATERAL = 10.0;  // Proportional gain
constexpr float KI_LATERAL = 0.0;   // Integral gain
constexpr float KD_LATERAL = 3.0;   // Derivative gain

// Angular PID controller
constexpr float KP_ANGULAR = 1;  // Proportional gain
constexpr float KI_ANGULAR = 0.0;  // Integral gain
constexpr float KD_ANGULAR = 3;  // Derivative gain

// Lift PID controller
constexpr float KP_LIFT = 0.5;  // Proportional gain
constexpr float KI_LIFT = 0.0;  // Integral gain
constexpr float KD_LIFT = 0.1;  // Derivative gaiN