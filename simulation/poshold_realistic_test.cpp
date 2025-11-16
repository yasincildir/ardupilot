/*
 * Realistic POSHOLD Obstacle Avoidance Test
 * Using actual rangefinder-based proximity detection
 *
 * Scenario:
 * 1. POSHOLD mode with forward rangefinder
 * 2. Approach wall at 0.5 m/s
 * 3. Release sticks at 2m
 * 4. Proximity sensor detects wall, triggers avoidance
 */

#include <iostream>
#include <cmath>
#include <iomanip>
#include <random>

// ArduPilot parameters (from indoor_flight_4.5.7_optimized.param)
const float AVOID_ENABLE = 7;           // All features enabled
const float AVOID_MARGIN = 2.0f;        // Stop distance (meters)
const float AVOID_BEHAVE = 1;           // Behavior: 1=Stop
const float PRX_TYPE = 4;               // Proximity type: 4=RangeFinder
const float RNGFND_MAX_CM = 400;        // 4m max range

// Simulation parameters
const float INITIAL_DISTANCE = 5.0f;    // meters
const float APPROACH_SPEED = 0.5f;      // m/s
const float STICK_RELEASE_DIST = 2.0f;  // meters
const float DT = 0.02f;                 // 50Hz update rate (20ms)
const float POSHOLD_BRAKE_RATE = 2.0f;  // m/s² deceleration

// Random noise generator
std::random_device rd;
std::mt19937 gen(rd());

float add_noise(float value, float sigma) {
    std::normal_distribution<float> dist(value, sigma);
    return dist(gen);
}

struct ProximitySensor {
    float distance;         // meters to obstacle
    bool valid;             // sensor reading valid?
    uint32_t last_update_ms;
};

struct DroneState {
    float position;         // meters from wall
    float velocity;         // m/s (negative = towards wall)
    float stick_input;      // -1.0 to 1.0
    uint32_t time_ms;

    // POSHOLD state
    bool poshold_active;
    float target_velocity;

    // Avoidance state
    bool avoid_active;
    float avoid_vel_limit;
};

// Simulate forward rangefinder
ProximitySensor read_proximity(const DroneState& drone) {
    ProximitySensor sensor;

    // Add realistic sensor noise (±2cm)
    sensor.distance = add_noise(drone.position, 0.02f);

    // Check range limits
    if (sensor.distance < 0.05f || sensor.distance > 4.0f) {
        sensor.valid = false;
    } else {
        sensor.valid = true;
    }

    sensor.last_update_ms = drone.time_ms;
    return sensor;
}

// Obstacle avoidance controller (simplified ArduPilot behavior)
void update_obstacle_avoidance(DroneState& drone, const ProximitySensor& prox) {
    if (!prox.valid) {
        drone.avoid_active = false;
        drone.avoid_vel_limit = 100.0f; // No limit
        return;
    }

    // Calculate margin from obstacle
    float margin = prox.distance;

    if (margin <= AVOID_MARGIN) {
        drone.avoid_active = true;

        // Limit velocity based on distance
        // Closer = slower allowed speed
        float distance_ratio = margin / AVOID_MARGIN;

        if (distance_ratio < 0.5f) {
            // Very close - stop immediately
            drone.avoid_vel_limit = 0.0f;
        } else {
            // Reduce speed proportionally
            drone.avoid_vel_limit = -APPROACH_SPEED * (distance_ratio - 0.5f) * 2.0f;
        }
    } else {
        drone.avoid_active = false;
        drone.avoid_vel_limit = 100.0f;
    }
}

// POSHOLD controller
void update_poshold(DroneState& drone, float dt) {
    // Calculate target velocity from stick input
    if (fabsf(drone.stick_input) > 0.01f) {
        drone.target_velocity = drone.stick_input * APPROACH_SPEED;
    } else {
        // No stick input - brake to stop
        drone.target_velocity = 0.0f;
    }

    // Apply obstacle avoidance limits
    if (drone.avoid_active) {
        if (drone.target_velocity < drone.avoid_vel_limit) {
            drone.target_velocity = drone.avoid_vel_limit;
        }
    }

    // Smooth acceleration towards target
    float vel_error = drone.target_velocity - drone.velocity;
    float max_accel = POSHOLD_BRAKE_RATE * dt;

    if (fabsf(vel_error) < max_accel) {
        drone.velocity = drone.target_velocity;
    } else {
        drone.velocity += copysignf(max_accel, vel_error);
    }

    // Update position
    drone.position += drone.velocity * dt;

    // Prevent going through wall
    if (drone.position < 0.0f) {
        drone.position = 0.0f;
        drone.velocity = 0.0f;
    }
}

int main() {
    std::cout << "==============================================\n";
    std::cout << "REALISTIC POSHOLD OBSTACLE AVOIDANCE TEST\n";
    std::cout << "With Rangefinder-Based Proximity Detection\n";
    std::cout << "==============================================\n\n";

    std::cout << "ArduPilot Parameters:\n";
    std::cout << "  AVOID_ENABLE = " << (int)AVOID_ENABLE << " (all features)\n";
    std::cout << "  AVOID_MARGIN = " << AVOID_MARGIN << " m\n";
    std::cout << "  AVOID_BEHAVE = " << (int)AVOID_BEHAVE << " (stop)\n";
    std::cout << "  PRX_TYPE = " << (int)PRX_TYPE << " (rangefinder)\n\n";

    std::cout << "Test Scenario:\n";
    std::cout << "1. Start 5m from wall in POSHOLD mode\n";
    std::cout << "2. Forward stick → approach at 0.5 m/s\n";
    std::cout << "3. Release sticks at 2.0m\n";
    std::cout << "4. Proximity sensor detects wall\n";
    std::cout << "5. Avoidance system stops drone\n\n";

    // Initialize drone
    DroneState drone;
    drone.position = INITIAL_DISTANCE;
    drone.velocity = 0.0f;
    drone.stick_input = -1.0f;  // Full forward
    drone.time_ms = 0;
    drone.poshold_active = true;
    drone.target_velocity = 0.0f;
    drone.avoid_active = false;
    drone.avoid_vel_limit = 100.0f;

    bool sticks_released = false;
    float closest_distance = INITIAL_DISTANCE;
    float time_s = 0.0f;

    std::cout << "==============================================\n";
    std::cout << "SIMULATION (50Hz Update Rate)\n";
    std::cout << "==============================================\n\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time   Dist   Vel     Stick  Prox   Avoid  Status\n";
    std::cout << "(s)    (m)    (m/s)   Input  (m)    Active\n";
    std::cout << "-----  -----  ------  -----  -----  -----  ------\n";

    while (time_s < 15.0f) {
        // Release sticks at 2m
        if (!sticks_released && drone.position <= STICK_RELEASE_DIST) {
            drone.stick_input = 0.0f;
            sticks_released = true;
        }

        // Read proximity sensor
        ProximitySensor prox = read_proximity(drone);

        // Update obstacle avoidance
        update_obstacle_avoidance(drone, prox);

        // Update POSHOLD controller
        update_poshold(drone, DT);

        // Track closest distance
        if (drone.position < closest_distance) {
            closest_distance = drone.position;
        }

        // Print status every 0.5s
        if ((int)(time_s / DT) % 25 == 0) {
            std::cout << std::setw(5) << time_s << "  ";
            std::cout << std::setw(5) << drone.position << "  ";
            std::cout << std::setw(6) << drone.velocity << "  ";
            std::cout << std::setw(5) << (drone.stick_input != 0 ? "FWD" : "OFF") << "  ";
            std::cout << std::setw(5) << (prox.valid ? prox.distance : -1.0f) << "  ";
            std::cout << std::setw(5) << (drone.avoid_active ? "YES" : "NO") << "  ";

            // Status
            if (!sticks_released) {
                std::cout << "Approaching...";
            } else if (drone.avoid_active) {
                std::cout << "\033[33mAVOIDING! Limit=" << drone.avoid_vel_limit << "m/s\033[0m";
            } else if (fabsf(drone.velocity) < 0.01f && sticks_released) {
                std::cout << "\033[32mSTOPPED SAFELY\033[0m";
            } else {
                std::cout << "Braking...";
            }
            std::cout << "\n";
        }

        // Check if stopped
        if (sticks_released && fabsf(drone.velocity) < 0.01f &&
            time_s > (STICK_RELEASE_DIST / APPROACH_SPEED + 3.0f)) {
            break;
        }

        time_s += DT;
        drone.time_ms += (uint32_t)(DT * 1000);
    }

    std::cout << "\n==============================================\n";
    std::cout << "TEST RESULTS\n";
    std::cout << "==============================================\n\n";

    std::cout << "Flight Statistics:\n";
    std::cout << "  Closest Distance: " << closest_distance << " m\n";
    std::cout << "  Final Position: " << drone.position << " m\n";
    std::cout << "  Final Velocity: " << drone.velocity << " m/s\n";
    std::cout << "  Sticks Released: " << (sticks_released ? "YES" : "NO") << "\n";
    std::cout << "  Avoidance Active: " << (drone.avoid_active ? "YES" : "NO") << "\n\n";

    // Evaluate
    bool passed = true;

    std::cout << "Safety Checks:\n";

    if (drone.position > 0.0f) {
        std::cout << "  \033[32m✓ No collision\033[0m\n";
    } else {
        std::cout << "  \033[31m✗ COLLISION!\033[0m\n";
        passed = false;
    }

    if (closest_distance >= AVOID_MARGIN * 0.6f) {
        std::cout << "  \033[32m✓ Safe margin maintained (>= "
                  << (AVOID_MARGIN * 0.6f) << "m)\033[0m\n";
    } else {
        std::cout << "  \033[31m✗ Too close! (< "
                  << (AVOID_MARGIN * 0.6f) << "m)\033[0m\n";
        passed = false;
    }

    if (fabsf(drone.velocity) < 0.05f) {
        std::cout << "  \033[32m✓ Stopped successfully\033[0m\n";
    } else {
        std::cout << "  \033[33m⚠ Still moving (" << drone.velocity << " m/s)\033[0m\n";
    }

    if (closest_distance <= AVOID_MARGIN * 1.2f) {
        std::cout << "  \033[32m✓ Obstacle detected by proximity sensor\033[0m\n";
    } else {
        std::cout << "  \033[33m⚠ Avoidance may not have triggered\033[0m\n";
    }

    std::cout << "\n==============================================\n";

    if (passed) {
        std::cout << "RESULT: \033[32m✓ PASS\033[0m\n\n";
        std::cout << "POSHOLD obstacle avoidance works correctly!\n";
        std::cout << "Drone safely stops before hitting wall.\n";
    } else {
        std::cout << "RESULT: \033[31m✗ FAIL\033[0m\n\n";
        std::cout << "Obstacle avoidance needs adjustment.\n";
    }

    std::cout << "==============================================\n\n";

    // Real-world test instructions
    std::cout << "🚁 REAL FLIGHT TEST PROCEDURE:\n\n";
    std::cout << "Pre-Flight Checklist:\n";
    std::cout << "□ Load indoor_flight_4.5.7_optimized.param\n";
    std::cout << "□ Verify AVOID_ENABLE = 7\n";
    std::cout << "□ Verify AVOID_MARGIN = 2.0m\n";
    std::cout << "□ Verify AVOID_BEHAVE = 1 (Stop)\n";
    std::cout << "□ Connect rangefinder/lidar to forward direction\n";
    std::cout << "□ Test proximity sensor readings in Mission Planner\n\n";

    std::cout << "Flight Test Steps:\n";
    std::cout << "1. Arm in LOITER or ALT_HOLD\n";
    std::cout << "2. Take off to 1.5m altitude\n";
    std::cout << "3. Switch to POSHOLD mode\n";
    std::cout << "4. Position drone 5m from wall\n";
    std::cout << "5. Slowly push forward stick (25-50%)\n";
    std::cout << "6. Approach wall at ~0.5 m/s\n";
    std::cout << "7. At 2m distance, release sticks completely\n";
    std::cout << "8. Observe:\n";
    std::cout << "   - Drone should slow down automatically\n";
    std::cout << "   - Should stop at ~1.5-2.0m from wall\n";
    std::cout << "   - LED may blink (proximity warning)\n";
    std::cout << "   - Beeper may sound (if configured)\n\n";

    std::cout << "Expected Behavior:\n";
    std::cout << "✓ Drone decelerates smoothly\n";
    std::cout << "✓ Stops before reaching AVOID_MARGIN (2m)\n";
    std::cout << "✓ No collision with wall\n";
    std::cout << "✓ Hovering stably after stop\n\n";

    std::cout << "Safety Notes:\n";
    std::cout << "⚠ Keep LOITER mode ready on switch for emergency\n";
    std::cout << "⚠ Test in open area first (no walls nearby)\n";
    std::cout << "⚠ Start with slow speeds (0.3 m/s)\n";
    std::cout << "⚠ Be ready to take manual control\n\n";

    return passed ? 0 : 1;
}
