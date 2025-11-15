#include "Copter.h"

// ========================================
// Indoor Altitude Hold - Obstacle Detection
// ========================================

// Obstacle detection thresholds and constants
#define OBSTACLE_DETECTION_ENABLED 1
#define OBSTACLE_HYSTERESIS_SAMPLES 5           // Require 5 consecutive samples to confirm floor change
#define FLOOR_TRACKING_TAU 0.1f                 // Time constant for floor height smoothing (100ms)
#define TILT_AGGRESSIVE_THRESHOLD 0.87f         // cos(30°) - use aggressive detection when tilted >30°

// Note: INDOOR_OBS_THR and INDOOR_FLR_RATE are now runtime parameters (g2.indoor_obs_thr, g2.indoor_floor_rate)

// Helper function: Detects obstacles vs floor changes using rate-of-change and hysteresis
// Returns true if current measurement is likely an obstacle (should be filtered out)
static bool detect_obstacle_and_track_floor(
    float current_alt_m,
    float& floor_height_estimate_m,
    int8_t& obstacle_counter,
    uint32_t& last_floor_update_ms,
    uint32_t now_ms,
    float dt,
    float tilt_correction,
    float obstacle_jump_threshold_m,
    float max_floor_change_rate_ms)
{
    if (!OBSTACLE_DETECTION_ENABLED) {
        return false;
    }

    // Tilt-aware threshold adjustment
    // When vehicle is tilted >30°, use more aggressive detection (lower thresholds)
    float jump_threshold = obstacle_jump_threshold_m;
    float max_floor_rate = max_floor_change_rate_ms;

    if (tilt_correction <= TILT_AGGRESSIVE_THRESHOLD) {
        jump_threshold *= 0.7f;      // 0.8m → 0.56m
        max_floor_rate *= 0.7f;      // 0.3 m/s → 0.21 m/s
    }

    // Initialize floor estimate on first run
    if (last_floor_update_ms == 0) {
        floor_height_estimate_m = current_alt_m;
        last_floor_update_ms = now_ms;
        obstacle_counter = 0;
        return false;
    }

    const uint32_t time_since_update_ms = now_ms - last_floor_update_ms;
    if (time_since_update_ms == 0) {
        return false;  // Avoid division by zero
    }
    const float time_since_update_s = time_since_update_ms * 0.001f;

    const float delta_m = current_alt_m - floor_height_estimate_m;

    // Check if this is a significant jump
    if (fabsf(delta_m) > jump_threshold) {
        // Calculate rate of change
        const float floor_change_rate_ms = fabsf(delta_m) / time_since_update_s;

        if (floor_change_rate_ms > max_floor_rate) {
            // Too fast = likely obstacle
            obstacle_counter = MIN(obstacle_counter + 1, OBSTACLE_HYSTERESIS_SAMPLES + 1);
        } else {
            // Gradual change = likely real floor change
            obstacle_counter = MAX(obstacle_counter - 1, -(OBSTACLE_HYSTERESIS_SAMPLES + 1));
        }

        // Hysteresis: require consecutive samples to confirm
        if (obstacle_counter >= OBSTACLE_HYSTERESIS_SAMPLES) {
            // Confirmed obstacle - keep returning true
            return true;
        } else if (obstacle_counter <= -OBSTACLE_HYSTERESIS_SAMPLES) {
            // Confirmed gradual floor change - accept it
            floor_height_estimate_m = current_alt_m;
            last_floor_update_ms = now_ms;
            obstacle_counter = 0;
            return false;
        } else {
            // Still uncertain - filter out while deciding
            return true;
        }
    } else {
        // Small change - smooth track the floor
        obstacle_counter = 0;  // Reset hysteresis

        // Low-pass filter: new = old + alpha * (measurement - old)
        const float alpha = dt / (dt + FLOOR_TRACKING_TAU);
        floor_height_estimate_m += alpha * delta_m;
        last_floor_update_ms = now_ms;
        return false;
    }
}

// update_surface_offset - manages the vertical offset of the position controller to follow the measured ground or ceiling
//   level measured using the range finder.
void Copter::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    // check for timeout
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // check tracking state and that range finders are healthy
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // calculate surfaces height above the EKF origin
        // e.g. if vehicle is 10m above the EKF origin and rangefinder reports alt of 3m.  curr_surface_alt_above_origin_cm is 7m (or 700cm)
        RangeFinderState &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;

        // Indoor Altitude Hold - Obstacle Detection
        // Apply obstacle detection for ground tracking only
        if (surface == Surface::GROUND) {
            // Get current altitude in meters
            const float current_alt_m = rf_state.alt_cm * 0.01f;

            // Get tilt correction factor (same as in sensors.cpp)
#if RANGEFINDER_TILT_CORRECTION == ENABLED
            const float tilt_correction = MAX(0.707f, copter.ahrs.get_rotation_body_to_ned().c.z);
#else
            const float tilt_correction = 1.0f;
#endif

            // Check if this is an obstacle (using runtime parameters)
            const bool is_obstacle = detect_obstacle_and_track_floor(
                current_alt_m,
                rf_state.floor_height_estimate_m,
                rf_state.obstacle_counter,
                rf_state.last_floor_update_ms,
                now_ms,
                0.05f,  // Assuming 20Hz update rate (50ms)
                tilt_correction,
                copter.g2.indoor_obs_thr,       // Runtime parameter: obstacle jump threshold
                copter.g2.indoor_floor_rate     // Runtime parameter: max floor change rate
            );

            // If detected as obstacle, use floor estimate instead of raw measurement
            if (is_obstacle) {
                // Recalculate terrain_offset_cm using floor estimate
                const float floor_alt_cm = rf_state.floor_height_estimate_m * 100.0f;
                rf_state.terrain_offset_cm = rf_state.inertial_alt_cm - floor_alt_cm;
            }
        }

        // update position controller target offset to the surface's alt above the EKF origin
        copter.pos_control->set_pos_offset_target_z_cm(rf_state.terrain_offset_cm);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // reset target altitude if this controller has just been engaged
        // target has been changed between upwards vs downwards
        // or glitch has cleared
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->set_pos_offset_z_cm(rf_state.terrain_offset_cm);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // reset position controller offsets if surface tracking is inactive
        // flag target should be reset when/if it next becomes active
        if (timeout) {
            copter.pos_control->set_pos_offset_z_cm(0);
            copter.pos_control->set_pos_offset_target_z_cm(0);
            reset_target = true;
        }
    }
#else
    copter.pos_control->set_pos_offset_z_cm(0);
    copter.pos_control->set_pos_offset_target_z_cm(0);
#endif
}


// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Copter::SurfaceTracking::get_target_alt_cm(float &target_alt_cm) const
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return false;
    }
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    target_alt_cm = (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm());
    return true;
}

// set target altitude (in cm) above ground
void Copter::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return;
    }
    copter.pos_control->set_pos_offset_z_cm(copter.inertial_nav.get_position_z_up_cm() - _target_alt_cm);
    last_update_ms = AP_HAL::millis();
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist = dir * (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm()) * 0.01f;
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_cm : copter.rangefinder_state.alt_cm) * 0.01f;
}

// set direction
void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    if (surface == new_surface) {
        return;
    }
    // check we have a range finder in the correct direction
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    surface = new_surface;
    reset_target = true;
}
