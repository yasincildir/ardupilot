# Indoor Altitude Transition - Compile-Time Feature Control

## Overview

The Indoor Altitude Transition feature handles GPS↔Indoor altitude reference changes for multi-floor indoor navigation. This is essential when entering buildings at different floor levels (e.g., entering 4th floor from outdoor).

**Current Status:** Feature is **ENABLED by default** (safe for all flights)

## Feature Behavior

### When ENABLED (Default)
- Detects altitude reference changes (GPS MSL ↔ Indoor AGL)
- Threshold: 5m rangefinder change
- IMU validation: 0.5 m/s² vertical acceleration check
- Smooth 3-second linear blend between altitude references
- Temporarily expands barometer innovation gate (700→10000)
- **Safe for both indoor and outdoor flights**

### When DISABLED
- Feature completely removed at compile time (zero code overhead)
- Normal ArduPilot behavior
- No altitude transition detection or blending
- Use this for outdoor-only flights or if you encounter issues

## How to Enable/Disable

### Location
File: `libraries/AP_NavEKF3/AP_NavEKF3_feature.h` (around line 74-76)

**IMPORTANT:** This file contains ALL EKF3 feature flags (beacon fusion, optical flow, etc.). Only modify the indoor altitude transition section at the bottom of the file.

### To DISABLE (Outdoor-only flights)
Change line 75 in `AP_NavEKF3_feature.h`:
```cpp
#ifndef EKF3_INDOOR_ALT_TRANSITION_ENABLED
#define EKF3_INDOOR_ALT_TRANSITION_ENABLED 0  // Changed from 1 to 0
#endif
```

### To ENABLE (Indoor flight support)
Keep default value (line 75):
```cpp
#ifndef EKF3_INDOOR_ALT_TRANSITION_ENABLED
#define EKF3_INDOOR_ALT_TRANSITION_ENABLED 1  // Default value
#endif
```

### After Changing
```bash
./waf copter          # Rebuild copter firmware
./waf copter --upload # Upload to autopilot
```

## Use Cases

### When to ENABLE (Default)
- ✅ Multi-floor indoor navigation
- ✅ Entering buildings at different floor levels
- ✅ Mixed outdoor/indoor flights
- ✅ Flights over tables, furniture, terrain changes
- ✅ General purpose (works for all scenarios)

### When to DISABLE
- ❌ Outdoor-only flights (no indoor navigation needed)
- ❌ Troubleshooting altitude issues
- ❌ Minimal code size requirements
- ❌ Testing against vanilla ArduPilot behavior

## Technical Details

### Detection Threshold
The 5m threshold is hard-coded to prevent false triggers from small obstacles:

| Scenario | Rangefinder Change | Triggered? |
|----------|-------------------|------------|
| 0.8m table crossing | 0.8m | ❌ No (< 5m) |
| 3m furniture | 3m | ❌ No (< 5m) |
| 4th floor entry (138m MSL) | ~138m | ✅ Yes (> 5m) |
| Multi-story transition | 10m+ | ✅ Yes (> 5m) |

**This is by design!** Small obstacles (< 5m) should NOT trigger altitude reference changes.

### IMU Validation
Before triggering transition, the system checks vertical acceleration:
- If `|accel_z + 9.8| > 0.5 m/s²` → Real drone movement, don't transition
- If `|accel_z + 9.8| ≤ 0.5 m/s²` → Altitude reference change, start transition

This prevents false triggers during rapid ascent/descent.

### Code Locations (For Developers)

Modified files:
1. `libraries/AP_NavEKF3/AP_NavEKF3_feature.h` - Feature configuration
2. `libraries/AP_NavEKF3/AP_NavEKF3_core.h` - State variables (lines 1552-1562)
3. `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` - Blend logic (lines 735-753)
4. `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` - Detection (lines 88-126)

All code is wrapped with:
```cpp
#if EKF3_INDOOR_ALT_TRANSITION_ENABLED
    // ... feature code ...
#endif
```

## Comparison with Original ArduPilot

| Aspect | Original ArduPilot | With Feature ENABLED | With Feature DISABLED |
|--------|-------------------|---------------------|----------------------|
| Code size | Baseline | +~150 bytes | Identical to original |
| CPU overhead | N/A | Minimal (5m check) | None |
| GPS→Indoor entry | May diverge/reject | Smooth 3s blend | May diverge/reject |
| Small obstacles | Normal | Normal | Normal |
| Multi-floor support | No | Yes | No |
| Barometer behavior | Normal | Gate expanded during transition | Normal |

## Parameters Used

**NONE!** This feature uses hard-coded values to avoid boot freeze issues encountered in previous implementations.

All values are embedded in the code:
- Threshold: 5m (hard-coded in AP_NavEKF3_Measurements.cpp:95)
- IMU validation: 0.5 m/s² (hard-coded in AP_NavEKF3_Measurements.cpp:106)
- Blend duration: 3000ms (hard-coded in AP_NavEKF3_Measurements.cpp:117)
- Baro gate: 10000 (hard-coded in AP_NavEKF3_Measurements.cpp:121)

To change these values, modify the source code and rebuild.

## Safety Notes

1. **Default is ENABLED** - This is intentional. The 5m threshold makes it safe for all flights.
2. **No parameters** - Cannot be changed at runtime (by design, prevents boot freeze)
3. **Compile-time only** - Must rebuild firmware to enable/disable
4. **Zero overhead when disabled** - Preprocessor completely removes code
5. **Works with EK3_HGT_I_GATE** - Existing parameter still controls normal baro gate

## Troubleshooting

### "Altitude jumping during indoor entry"
- Feature is likely **disabled**. Check AP_NavEKF3_feature.h, set to 1, rebuild.

### "Altitude transitioning on small obstacles"
- Feature is working, but threshold may be too low
- Current 5m threshold should prevent this
- Check rangefinder readings during the event

### "Want to disable for testing"
- Set `EKF3_INDOOR_ALT_TRANSITION_ENABLED 0` in AP_NavEKF3_feature.h
- Rebuild firmware: `./waf copter`
- Upload to autopilot

### "Boot freeze after enabling"
- This feature does NOT use parameters (no boot freeze risk)
- If experiencing boot freeze, check for other modifications

## Version History

- **2024**: Initial implementation with parameters (caused boot freeze)
- **2024**: Removed parameters, hard-coded values
- **2024**: Added compile-time enable/disable switch (this version)

## Related Parameters

This feature does NOT add any new parameters. It works with existing ArduPilot parameters:

- `EK3_HGT_I_GATE` - Normal barometer innovation gate (default: 700, recommended: 700)
- `EKF3_AFFINITY` - EKF sensor affinity
- `RNGFND1_TYPE` - Rangefinder type
- `RNGFND1_MAX_CM` - Rangefinder maximum range (recommend 3000cm for 30m lidar)

## Support

For issues, questions, or modifications, refer to the ArduPilot indoor flight development branch.
