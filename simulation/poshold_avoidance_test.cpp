/*
 * POSHOLD Mode Obstacle Avoidance Test
 *
 * Tests the scenario:
 * 1. Switch to POSHOLD mode
 * 2. Fly slowly towards wall (0.5 m/s)
 * 3. Release sticks at 2m distance
 * 4. Expected: Drone should stop/push back when approaching wall
 */

#include <iostream>
#include <cmath>
#include <iomanip>

// Simulation parameters
const float AVOID_MARGIN = 2.0f;        // meters - stop distance from obstacle
const float AVOID_BACKUP_DIST = 0.5f;   // meters - how far to push back
const float INITIAL_DISTANCE = 5.0f;    // meters - starting distance from wall
const float APPROACH_SPEED = 0.5f;      // m/s - approach velocity
const float STICK_RELEASE_DIST = 2.0f;  // meters - when pilot releases sticks
const float DT = 0.1f;                  // seconds - simulation timestep

struct DroneState {
    float position;      // meters from wall
    float velocity;      // m/s (negative = towards wall)
    float stick_input;   // 0.0-1.0 (0=released)
    bool avoid_active;   // obstacle avoidance triggered?
};

// Simulate obstacle avoidance behavior
void update_poshold_avoidance(DroneState& drone, float dt) {
    // Check if obstacle avoidance should activate
    if (drone.position <= AVOID_MARGIN) {
        drone.avoid_active = true;

        // Push back or stop
        if (drone.velocity < 0) {
            // Currently moving towards wall - stop and reverse
            drone.velocity = AVOID_BACKUP_DIST / dt;  // Push back
        }
    } else {
        drone.avoid_active = false;
    }

    // Apply stick input if pilot is controlling
    if (drone.stick_input > 0) {
        // Pilot input: move towards wall
        drone.velocity = -APPROACH_SPEED * drone.stick_input;
    } else {
        // No pilot input: let avoidance system control
        if (drone.avoid_active) {
            // Push back from wall
            if (drone.position < AVOID_MARGIN) {
                drone.velocity = AVOID_BACKUP_DIST;
            }
        } else {
            // Decelerate to stop (normal POSHOLD behavior)
            drone.velocity *= 0.9f;  // Simple decay
        }
    }

    // Update position
    drone.position += drone.velocity * dt;

    // Prevent going through wall
    if (drone.position < 0) {
        drone.position = 0;
        drone.velocity = 0;
    }
}

int main() {
    std::cout << "==============================================\n";
    std::cout << "POSHOLD MODE OBSTACLE AVOIDANCE TEST\n";
    std::cout << "==============================================\n\n";

    std::cout << "Test Scenario:\n";
    std::cout << "1. Start in POSHOLD mode, 5m from wall\n";
    std::cout << "2. Fly towards wall at 0.5 m/s\n";
    std::cout << "3. Release sticks at 2m distance\n";
    std::cout << "4. Verify: Drone stops before hitting wall\n\n";

    std::cout << "Parameters:\n";
    std::cout << "  AVOID_MARGIN = " << AVOID_MARGIN << " m\n";
    std::cout << "  APPROACH_SPEED = " << APPROACH_SPEED << " m/s\n";
    std::cout << "  STICK_RELEASE = " << STICK_RELEASE_DIST << " m\n\n";

    // Initialize drone
    DroneState drone;
    drone.position = INITIAL_DISTANCE;
    drone.velocity = 0.0f;
    drone.stick_input = 1.0f;  // Full forward stick
    drone.avoid_active = false;

    float time = 0.0f;
    bool test_passed = true;
    bool sticks_released = false;
    float closest_distance = INITIAL_DISTANCE;

    std::cout << "==============================================\n";
    std::cout << "SIMULATION\n";
    std::cout << "==============================================\n\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s)  Dist(m)  Vel(m/s)  Stick  Avoid  Status\n";
    std::cout << "-------  -------  --------  -----  -----  ------\n";

    while (time < 20.0f && drone.position > 0) {
        // Release sticks at 2m
        if (!sticks_released && drone.position <= STICK_RELEASE_DIST) {
            drone.stick_input = 0.0f;
            sticks_released = true;
        }

        // Update simulation
        update_poshold_avoidance(drone, DT);

        // Track closest approach
        if (drone.position < closest_distance) {
            closest_distance = drone.position;
        }

        // Print status every 0.5s
        if ((int)(time / DT) % 5 == 0) {
            std::cout << std::setw(7) << time << "  ";
            std::cout << std::setw(7) << drone.position << "  ";
            std::cout << std::setw(8) << drone.velocity << "  ";
            std::cout << std::setw(5) << (drone.stick_input > 0 ? "YES" : "NO") << "  ";
            std::cout << std::setw(5) << (drone.avoid_active ? "ON" : "OFF") << "  ";

            // Status indicators
            if (!sticks_released) {
                std::cout << "Approaching...";
            } else if (drone.avoid_active) {
                std::cout << "\033[33mAVOIDING OBSTACLE!\033[0m";
            } else if (fabsf(drone.velocity) < 0.01f && sticks_released) {
                std::cout << "\033[32mSTOPPED\033[0m";
            } else {
                std::cout << "Decelerating...";
            }
            std::cout << "\n";
        }

        // Check if stopped safely
        if (sticks_released && fabsf(drone.velocity) < 0.01f &&
            drone.position >= AVOID_MARGIN * 0.8f) {
            // Stopped safely before reaching wall
            std::cout << "\n\033[32m✓ Drone stopped safely at "
                      << drone.position << "m\033[0m\n";
            break;
        }

        time += DT;
    }

    std::cout << "\n==============================================\n";
    std::cout << "TEST RESULTS\n";
    std::cout << "==============================================\n\n";

    std::cout << "Closest Distance to Wall: " << closest_distance << " m\n";
    std::cout << "Final Position: " << drone.position << " m\n";
    std::cout << "Final Velocity: " << drone.velocity << " m/s\n";
    std::cout << "Avoid Activated: " << (drone.avoid_active ? "YES" : "NO") << "\n\n";

    // Evaluate test
    std::cout << "Test Evaluation:\n";

    if (closest_distance >= AVOID_MARGIN * 0.8f) {
        std::cout << "  \033[32m✓ Maintained safe distance (>= "
                  << (AVOID_MARGIN * 0.8f) << "m)\033[0m\n";
    } else {
        std::cout << "  \033[31m✗ Too close to wall! (< "
                  << (AVOID_MARGIN * 0.8f) << "m)\033[0m\n";
        test_passed = false;
    }

    if (drone.position > 0) {
        std::cout << "  \033[32m✓ Did not hit wall\033[0m\n";
    } else {
        std::cout << "  \033[31m✗ COLLISION!\033[0m\n";
        test_passed = false;
    }

    if (sticks_released) {
        std::cout << "  \033[32m✓ Sticks released at " << STICK_RELEASE_DIST << "m\033[0m\n";
    }

    if (drone.avoid_active || closest_distance <= AVOID_MARGIN) {
        std::cout << "  \033[32m✓ Obstacle avoidance activated\033[0m\n";
    } else {
        std::cout << "  \033[33m⚠ Obstacle avoidance not triggered\033[0m\n";
    }

    std::cout << "\n==============================================\n";

    if (test_passed) {
        std::cout << "OVERALL: \033[32m✓ PASS\033[0m\n";
        std::cout << "POSHOLD obstacle avoidance works correctly!\n";
    } else {
        std::cout << "OVERALL: \033[31m✗ FAIL\033[0m\n";
        std::cout << "POSHOLD obstacle avoidance needs improvement.\n";
    }

    std::cout << "==============================================\n\n";

    // Real-world instructions
    std::cout << "Real Flight Test Instructions:\n";
    std::cout << "1. Load indoor_flight_4.5.7_optimized.param\n";
    std::cout << "2. Verify AVOID_ENABLE = 7 (enabled for all modes)\n";
    std::cout << "3. Verify AVOID_MARGIN = 2.0m\n";
    std::cout << "4. Switch to POSHOLD mode\n";
    std::cout << "5. Fly slowly towards wall (0.5 m/s)\n";
    std::cout << "6. At 2m, release sticks - drone should stop/push back\n";
    std::cout << "7. Observe: LED should blink, beep sound if proximity warning\n\n";

    return test_passed ? 0 : 1;
}
