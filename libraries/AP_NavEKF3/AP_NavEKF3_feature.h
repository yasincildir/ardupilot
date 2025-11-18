/*
   Indoor Flight Feature Configuration for EKF3

   This file contains compile-time feature flags for indoor flight capabilities.
   Change values and rebuild to enable/disable features.
*/

#pragma once

// Indoor Altitude Transition
// Handles GPS↔Indoor altitude reference changes (e.g., 4th floor entry)
// Set to 0 to disable (outdoor-only flights)
// Set to 1 to enable (multi-floor indoor navigation)
// Default: 1 (enabled)
#ifndef EKF3_INDOOR_ALT_TRANSITION_ENABLED
#define EKF3_INDOOR_ALT_TRANSITION_ENABLED 1
#endif

/*
   Usage:

   To DISABLE (outdoor-only):
   - Change to: #define EKF3_INDOOR_ALT_TRANSITION_ENABLED 0
   - Build: ./waf copter

   To ENABLE (indoor flight):
   - Change to: #define EKF3_INDOOR_ALT_TRANSITION_ENABLED 1
   - Build: ./waf copter

   When disabled:
   - Zero code overhead (completely removed at compile time)
   - Normal ArduPilot behavior
   - No altitude transition logic

   When enabled:
   - Smooth GPS↔Indoor transitions
   - 5m threshold detection
   - IMU validation
   - 3 second blend
*/
