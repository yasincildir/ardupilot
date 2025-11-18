/*
  feature selection for EKF3
 */

#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Beacon/AP_Beacon_config.h>
#include <AP_AHRS/AP_AHRS_config.h>

// define for when to include all features
#define EK3_FEATURE_ALL APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) || APM_BUILD_TYPE(APM_BUILD_Replay)

// body odomotry (which includes wheel encoding) on rover or 2M boards
#ifndef EK3_FEATURE_BODY_ODOM
#define EK3_FEATURE_BODY_ODOM EK3_FEATURE_ALL || APM_BUILD_TYPE(APM_BUILD_Rover) || BOARD_FLASH_SIZE > 1024
#endif

// external navigation on 2M boards
#ifndef EK3_FEATURE_EXTERNAL_NAV
#define EK3_FEATURE_EXTERNAL_NAV EK3_FEATURE_ALL || BOARD_FLASH_SIZE > 1024
#endif

// drag fusion on 2M boards
#ifndef EK3_FEATURE_DRAG_FUSION
#define EK3_FEATURE_DRAG_FUSION EK3_FEATURE_ALL || BOARD_FLASH_SIZE > 1024
#endif

// Beacon Fusion if beacon data available
#ifndef EK3_FEATURE_BEACON_FUSION
#define EK3_FEATURE_BEACON_FUSION AP_BEACON_ENABLED
#endif

#ifndef EK3_FEATURE_POSITION_RESET
#define EK3_FEATURE_POSITION_RESET EK3_FEATURE_ALL || AP_AHRS_POSITION_RESET_ENABLED
#endif

/*
   Indoor Altitude Transition Feature
   Handles GPS↔Indoor altitude reference changes (e.g., 4th floor entry)

   Set to 0 to disable (outdoor-only flights)
   Set to 1 to enable (multi-floor indoor navigation)

   When enabled:
   - Detects 5m+ altitude reference changes
   - Validates with IMU (0.5 m/s² threshold)
   - Smooth 3-second linear blend
   - Temporarily expands barometer innovation gate

   When disabled:
   - Zero code overhead (completely removed at compile time)
   - Normal ArduPilot behavior

   Default: 1 (enabled) - Safe for all flights due to 5m threshold
*/
#ifndef EKF3_INDOOR_ALT_TRANSITION_ENABLED
#define EKF3_INDOOR_ALT_TRANSITION_ENABLED 1
#endif
